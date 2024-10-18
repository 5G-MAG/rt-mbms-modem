// 5G-MAG Reference Tools
// MBMS Modem Process
//
// Copyright (C) 2021 Klaus Kühnhammer (Österreichische Rundfunksender GmbH & Co KG)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

#include "SdrReader.h"
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Types.hpp>
#include <SoapySDR/Formats.hpp>

#include <boost/algorithm/string/join.hpp>

#include <chrono>
#include <cmath>
#include <csignal>

#include "spdlog/spdlog.h"

SdrReader:: ~SdrReader() {
  if (_sdr != nullptr) {
    auto sdr = (SoapySDR::Device*)_sdr;
    sdr->deactivateStream((SoapySDR::Stream*)_stream, 0, 0);
    sdr->closeStream((SoapySDR::Stream*)_stream);
    SoapySDR::Device::unmake( sdr );
  }

  if (_reading_from_file) {
    srsran_filesource_free(&file_source);
  }

  if (_writing_to_file) {
    srsran_filesink_free(&file_sink);
  }
}

void SdrReader::enumerateDevices()
{
  auto results = SoapySDR::Device::enumerate();
	SoapySDR::Kwargs::iterator it;

	for( int i = 0; i < results.size(); ++i)
	{
		printf("Device #%d:\n", i);
		for( it = results[i].begin(); it != results[i].end(); ++it)
		{
			printf("%s = %s\n", it->first.c_str(), it->second.c_str());
		}
		printf("\n\n");
	}

}

auto SdrReader::init(const std::string& device_args, const char* sample_file,
                         const char* write_sample_file, bool repeat_sample_file) -> bool {
  if (sample_file != nullptr) {
    if (0 == srsran_filesource_init(&file_source,
                                    const_cast<char*>(sample_file),
                                    SRSRAN_COMPLEX_FLOAT_BIN)) {
      _reading_from_file = true;
      _repeat_sample_file = repeat_sample_file;
    } else {
      spdlog::error("Could not open file {}", sample_file);
      return false;
    }
  } else {
    if (write_sample_file != nullptr) {
      if (0 == srsran_filesink_init(&file_sink,
                                    const_cast<char*>(write_sample_file),
                                    SRSRAN_COMPLEX_FLOAT_BIN)) {
        _writing_to_file = true;
      } else {
        spdlog::error("Could not open file {}", write_sample_file);
        return false;
      }
    }
    _device_args = SoapySDR::KwargsFromString(device_args);
    _sdr = SoapySDR::Device::make(_device_args);
    if (_sdr == nullptr)
    {
      spdlog::error("SoapySDR: failed to open device with args {}", device_args);
      return false;
    }
  }

  _cfg.lookupValue("modem.sdr.ringbuffer_size_ms", _buffer_ms);
  return true;
}

void SdrReader::init_buffer() {
  auto buffer_size = (unsigned int)ceil(_sampleRate/1000.0 * _buffer_ms);
  _buffer = std::make_unique<MultichannelRingbuffer>(sizeof(cf_t) * buffer_size, _rx_channels);
  _buffer_write = std::make_unique<MultichannelRingbuffer>(sizeof(cf_t) * buffer_size * 3, _rx_channels); // This is the buffer where we will store the samples to write a big chunk of samples instead many little ones. It's size is three times the main buffer.
  _buffer_ready = true;
}

void SdrReader::clear_buffer() {
  _buffer->clear();
  _buffer_write->clear();
  _high_watermark_reached = false;
}

auto SdrReader::set_antenna(const std::string& antenna, uint8_t idx) -> bool {
  auto sdr = (SoapySDR::Device*)_sdr;
  auto antenna_list = sdr->listAntennas(SOAPY_SDR_RX, idx);
  if (std::find(antenna_list.begin(), antenna_list.end(), antenna) != antenna_list.end()) {
    sdr->setAntenna( SOAPY_SDR_RX, idx, antenna);
    _antenna = sdr->getAntenna( SOAPY_SDR_RX, idx);
    return true;
  } else {
    spdlog::error("Unknown antenna \"{}\". Available: {}.", antenna, boost::algorithm::join(antenna_list, ", ") );
    return false;
  }
}

auto SdrReader::set_frequency(uint32_t frequency, uint8_t idx) -> bool {
  auto sdr = (SoapySDR::Device*)_sdr;
  sdr->setFrequency( SOAPY_SDR_RX, idx, frequency);
  return true;
}

auto SdrReader::set_filter_bw(uint32_t bandwidth, uint8_t idx) -> bool {
  auto sdr = (SoapySDR::Device*)_sdr;
  sdr->setBandwidth( SOAPY_SDR_RX, idx, bandwidth);
  return true;
}

auto SdrReader::set_sample_rate(uint32_t rate, uint8_t idx) -> bool {
  auto sdr = (SoapySDR::Device*)_sdr;
  sdr->setSampleRate( SOAPY_SDR_RX, idx, rate);
  return true;
}

auto SdrReader::set_gain(bool use_agc, double gain, uint8_t idx) -> bool {
  auto sdr = (SoapySDR::Device*)_sdr;
  if (sdr->hasGainMode(SOAPY_SDR_RX, idx)) {
//    spdlog::info("{} AGC", use_agc ? "Enabling" : "Disabling");
    sdr->setGainMode(SOAPY_SDR_RX, idx, use_agc);
  } else if (use_agc) {
//    spdlog::info("AGC is not supported by this device, please set gain manually");
  }
  auto gain_range = sdr->getGainRange(SOAPY_SDR_RX, idx);
  _min_gain = gain_range.minimum();
  _max_gain = gain_range.maximum();
  if (gain >= gain_range.minimum() && gain <= gain_range.maximum()) {
    sdr->setGain( SOAPY_SDR_RX, idx, gain);
    if (idx == 0) {
      _gain = sdr->getGain( SOAPY_SDR_RX, idx);
    }
    return true;
  } else {
    spdlog::error("Invalid gain setting {}. Allowed range is: {} - {}.", gain, gain_range.minimum(), gain_range.maximum());
    return false;
  }
}

auto SdrReader::tune(uint32_t frequency, uint32_t sample_rate,
    uint32_t bandwidth, double gain, const std::string& antenna, bool use_agc) -> bool {
  _frequency = frequency;
  _filterBw = bandwidth;
  _sampleRate = sample_rate;
  _use_agc = use_agc;

  init_buffer();

  if (_reading_from_file) {
    return true;
  }

  if (_sdr == nullptr) {
    return false;
  }

  spdlog::info("Tuning to {} MHz, filter bandwidth {} MHz, sample rate {}, gain {}, antenna path {} with AGC set to {}",
      frequency/1000000.0, bandwidth/1000000.0, sample_rate/1000000.0, gain, antenna, use_agc);

  auto sdr = (SoapySDR::Device*)_sdr;

  for (auto ch = 0; ch < _rx_channels; ch++) {
    set_antenna(antenna, ch);
    set_gain(_use_agc, gain, ch);
    set_frequency(frequency, ch);
    set_filter_bw(bandwidth, ch);
    set_sample_rate(sample_rate, ch);
  }

  _frequency = sdr->getFrequency( SOAPY_SDR_RX, 0);
  bandwidth = sdr->getBandwidth( SOAPY_SDR_RX, 0);
  _sampleRate = sdr->getSampleRate( SOAPY_SDR_RX, 0);

  spdlog::info("SDR tuned to {} MHz, filter bandwidth {} MHz, sample rate {}, gain {}, antenna path {}",
      _frequency/1000000.0, bandwidth/1000000.0, _sampleRate/1000000.0, _gain, _antenna);


  auto sensors = sdr->listSensors();
  if (std::find(sensors.begin(), sensors.end(), "lms7_temp") != sensors.end()) {
    _temp_sensor_available = true;
    _temp_sensor_key = "lms7_temp";
  }

  return true;
}

void SdrReader::start() {
  if (_sdr != nullptr) {
    auto sdr = (SoapySDR::Device*)_sdr;
    std::vector<size_t> channels(_rx_channels);
    for (auto ch = 0; ch < _rx_channels; ch++) {
      channels[ch] = ch;
    }
    sdr->setHardwareTime(0); // Set SDR timestamp to zero.
    _stream = sdr->setupStream( SOAPY_SDR_RX, SOAPY_SDR_CF32, channels, _device_args);
    if( _stream == nullptr)
    {
      spdlog::error("Failed to set up RX stream");
      SoapySDR::Device::unmake( sdr );
      return ;
    }
    sdr->activateStream( (SoapySDR::Stream*)_stream, SOAPY_SDR_HAS_TIME, 100000000, 0); // Delayed start of the SDR reception, to avoid overfloas at the beggining.
  }
  _running = true;

  // Start the reader thread and elevate its priority to realtime
  _readerThread = std::thread{&SdrReader::read, this};
  struct sched_param thread_param = {};
  thread_param.sched_priority = 50;
  _cfg.lookupValue("modem.sdr.reader_thread_priority_rt", thread_param.sched_priority);

  spdlog::debug("Launching sample reader thread with realtime scheduling priority {}", thread_param.sched_priority);

  int error = pthread_setschedparam(_readerThread.native_handle(), SCHED_RR, &thread_param);
  if (error != 0) {
    spdlog::warn("Cannot set reader thread priority to realtime: {}. Thread will run at default priority with a high probability of dropped samples and loss of synchronisation.", strerror(error));
  }
}

void SdrReader::stop() {
  _running = false;

  _readerThread.join();
  if (_sdr != nullptr) {
    auto sdr = (SoapySDR::Device*)_sdr;
    sdr->deactivateStream((SoapySDR::Stream*)_stream, 0, 0);
    sdr->closeStream((SoapySDR::Stream*)_stream);
  }

  clear_buffer();
}

void SdrReader::read() {
  std::array<void*, SRSRAN_MAX_CHANNELS> radio_buffers = { nullptr };
  while (_running) {
    int toRead = ceil(_sampleRate / 1000.0);
    //int toRead = 254;
    if (_buffer->free_size() < toRead * sizeof(cf_t)) {
      spdlog::debug("ringbuffer overflow");
      std::this_thread::sleep_for(std::chrono::microseconds(1000));
    } else {
      int read = 0;
      size_t writeable = 0;
      size_t writeable_write = 0;
      auto buffers = _buffer->write_head(&writeable);
      auto buffers_write = _buffer_write->write_head(&writeable_write);
      int writeable_samples = (int)floor(writeable / sizeof(cf_t));
      int writeable_write_samples = (int)floor(writeable_write / sizeof(cf_t));

      if (_reading_from_file) {
        std::chrono::steady_clock::time_point entered = {};
        entered = std::chrono::steady_clock::now();

        read = srsran_filesource_read_multi(&file_source, buffers.data(), std::min(writeable_samples, toRead), (int)_rx_channels);
        if ( read == 0  ) {
          if (_repeat_sample_file) {
            srsran_filesource_seek(&file_source, 0);
          } else {
            spdlog::info("EOF, exiting...");
            raise(SIGINT); //SIGINT to signal srsran that we want to exit.
          }
        }
        read = read / _rx_channels;
        int64_t required_time_us = (1000000.0/_sampleRate) * read;

        if (read > 0) {
          _buffer->commit( read * sizeof(cf_t) );
        }

        std::chrono::microseconds sleep = (std::chrono::microseconds(required_time_us) -
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - entered));
        std::this_thread::sleep_for(sleep);
      } else {
        auto sdr = (SoapySDR::Device*)_sdr;
        int flags = 0;
        long long time_ns = 0;

        read = sdr->readStream( (SoapySDR::Stream*)_stream, buffers.data(), std::min(writeable_samples, toRead), flags, time_ns);

        if (read> 0 ) {
          auto rbuff = buffers.data();
          
          if (_writing_to_file) { // Only if we are going to write into a file.
            auto wbuff = buffers_write.data();
            for (int i = 0; i < _rx_channels; i++) {
              memcpy(wbuff[i], rbuff[i], read * sizeof(cf_t)); // Copy the data in the input buffer to the toWrite buffer.
            _buffer_write->commit( read * sizeof(cf_t) ); // We used another ring buffer for the written of the samples, to not block a lot we only write to the disk when the ring buffer is at 95% of its capacity
            }
          }
          _buffer->commit( read * sizeof(cf_t) );
    
          if (_writing_to_file && _write_samples && _buffer_write->used_size() >= 0.95* _buffer_write->capacity()) {
            int toWrite = _buffer_write->used_size(); // We are going to storage all the info
            auto buff_to_write = _buffer_write->read_head(); // Gives the beggining of the buffer, it also puts _used and _head to 0, to start adding at the beggining again.
            srsran_filesink_write_multi(&file_sink, buff_to_write.data(), toWrite, (int)_rx_channels); // From the begginin of the buffer we write used_size data
          }
              spdlog::debug("buffer: commited {}, requested {}, writeable {}, writeable_write {}, flags {}", read, toRead, writeable_samples, writeable_write_samples, flags);
        }
        else {
          spdlog::error("readStream returned {}", read);
          _buffer->commit(0);
        }
      }
    }
  }
  spdlog::debug("Sample reader thread exited");
}

auto SdrReader::get_samples(cf_t* data[SRSRAN_MAX_CHANNELS], uint32_t nsamples, //NOLINT
                               srsran_timestamp_t *
                               /*rx_time*/) -> int {
  std::chrono::steady_clock::time_point entered = {};
  entered = std::chrono::steady_clock::now();

  int64_t required_time_us = (1000000.0/_sampleRate) * nsamples;
  size_t cnt = nsamples * sizeof(cf_t);

  if (_high_watermark_reached &&  _buffer->used_size() < (_sampleRate / 1000.0) * 10 * sizeof(cf_t)) {
    _high_watermark_reached = false;
  }

  if (!_high_watermark_reached) {
    while (_buffer->used_size() < (_sampleRate / 1000.0) * (_buffer_ms / 2.0) * sizeof(cf_t)) {
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    spdlog::debug("Filled ringbuffer to half capacity");
    _high_watermark_reached = true;
  }

  std::vector<char*> buffers(_rx_channels);
  for (auto ch = 0; ch < _rx_channels; ch++) {
    buffers[ch] = (char*)data[ch];
  }
  _buffer->read(buffers, cnt); // Copy from the ringbuffer to the data array. This also decreases _used.

  if (_buffer->used_size() < (_sampleRate / 1000.0) * (_buffer_ms / 4.0) * sizeof(cf_t)) {
    required_time_us += 500;
  } else {
    required_time_us -= 500;
  }

  spdlog::debug("took {}, read {} samples, adjusted required {} us, delta {} us, sleep adj {},  sleeping for {} us",
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - entered).count(),
      nsamples,
      std::chrono::microseconds(required_time_us).count(),
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - _last_read).count(),
      _sleep_adjustment,
      (std::chrono::microseconds(_sleep_adjustment + required_time_us) - std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - _last_read)).count());

  std::chrono::microseconds sleep = (std::chrono::microseconds(_sleep_adjustment + required_time_us) - std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - _last_read));

  if (sleep.count() > 0) {
    std::this_thread::sleep_for(sleep);
    _sleep_adjustment = 0;
  } else if (sleep.count() > -100000) {
    _sleep_adjustment = sleep.count();
  }

  _last_read = std::chrono::steady_clock::now();
  return 0;
}

auto SdrReader::get_buffer_level() -> double
{ 
  if (!_buffer_ready) { 
    return 0; 
  } 
  return static_cast<double>(_buffer->used_size()) / static_cast<double>(_buffer->capacity()); 
}
