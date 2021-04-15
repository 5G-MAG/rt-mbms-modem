// OBECA - Open Broadcast Edge Cache Appliance
// Receive Process
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

#include "spdlog/spdlog.h"

SdrReader:: ~SdrReader() {
  if (_sdr != nullptr) {
    auto sdr = (SoapySDR::Device*)_sdr;
    sdr->deactivateStream((SoapySDR::Stream*)_stream, 0, 0);
    sdr->closeStream((SoapySDR::Stream*)_stream);
    SoapySDR::Device::unmake( sdr );
  }

  if (_reading_from_file) {
    srslte_filesource_free(&file_source);
  }

  if (_writing_to_file) {
    srslte_filesink_free(&file_sink);
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
                         const char* write_sample_file) -> bool {
  if (sample_file != nullptr) {
    if (0 == srslte_filesource_init(&file_source,
                                    const_cast<char*>(sample_file),
                                    SRSLTE_COMPLEX_FLOAT_BIN)) {
      _reading_from_file = true;
    } else {
      spdlog::error("Could not open file {}", sample_file);
      return false;
    }
  } else {
    if (write_sample_file != nullptr) {
      if (0 == srslte_filesink_init(&file_sink,
                                    const_cast<char*>(write_sample_file),
                                    SRSLTE_COMPLEX_FLOAT_BIN)) {
        _writing_to_file = true;
      } else {
        spdlog::error("Could not open file {}", write_sample_file);
        return false;
      }
    }

    auto args = SoapySDR::KwargsFromString(device_args);
    _sdr = SoapySDR::Device::make(args);
    if (_sdr == nullptr)
    {
      spdlog::error("SoapySDR: failed to open device with args {}", device_args);
      return false;
    }
  }

  _cfg.lookupValue("sdr.ringbuffer_size_ms", _buffer_ms);
  unsigned int buffer_size = 16384/*15360*/ * _buffer_ms;
      // at 10MHz BW / 15.36Mhz sample rate
  int error = 0;
  error = _buffer.initialize(sizeof(cf_t) * buffer_size);
  if (error != 0) {
    spdlog::error("Cannot allocate ringbuffer: {}", errno);
    return false;
  }
  _buffer_ready = true;
  return true;
}

void SdrReader::clear_buffer() {
  _buffer.clear();
  _high_watermark_reached = false;
}

auto SdrReader::setSampleRate(unsigned /*sample_rate*/) -> bool {
  return _sdr != nullptr;
}

auto SdrReader::tune(uint32_t frequency, uint32_t sample_rate,
    uint32_t bandwidth, double gain, const std::string& antenna) -> bool {
  _antenna = antenna;
  _gain = gain;
  _frequency = frequency;
  _filterBw = bandwidth;

  _buffer.clear();
  _high_watermark_reached = false;

  if (_reading_from_file) {
    _sampleRate = sample_rate;
    return true;
  }

  if (_sdr == nullptr) {
    return false;
  }

  spdlog::info("Tuning to {} MHz, filter bandwidth {} MHz, sample rate {}, gain {}, antenna path {}",
      frequency/1000000.0, bandwidth/1000000.0, sample_rate/1000000.0, gain, antenna);

  auto sdr = (SoapySDR::Device*)_sdr;

  auto antenna_list = sdr->listAntennas(SOAPY_SDR_RX, 0);
  if (std::find(antenna_list.begin(), antenna_list.end(), antenna) != antenna_list.end()) {
    sdr->setAntenna( SOAPY_SDR_RX, 0, antenna);
  } else {
    spdlog::error("Unknown antenna \"{}\". Available: {}.", antenna, boost::algorithm::join(antenna_list, ", ") );
    return false;
  }

  auto gain_range = sdr->getGainRange(SOAPY_SDR_RX, 0);
  if (gain >= gain_range.minimum() && gain <= gain_range.maximum()) {
    sdr->setGain( SOAPY_SDR_RX, 0, gain);
  } else {
    spdlog::error("Invalid gain setting {}. Allowed range is: {} - {}.", gain, gain_range.minimum(), gain_range.maximum());
    return false;
  }

  sdr->setFrequency( SOAPY_SDR_RX, 0, frequency);
  sdr->setBandwidth( SOAPY_SDR_RX, 0, bandwidth);
  sdr->setSampleRate( SOAPY_SDR_RX, 0, sample_rate);

  _antenna = sdr->getAntenna( SOAPY_SDR_RX, 0);
  _gain = sdr->getGain( SOAPY_SDR_RX, 0);
  _frequency = sdr->getFrequency( SOAPY_SDR_RX, 0);
  bandwidth = sdr->getBandwidth( SOAPY_SDR_RX, 0);
  _sampleRate = sdr->getSampleRate( SOAPY_SDR_RX, 0);

  spdlog::info("SDR tuned to {} MHz, filter bandwidth {} MHz, sample rate {}, gain {}, antenna path {}",
      _frequency/1000000.0, bandwidth/1000000.0, _sampleRate/1000000.0, _gain, _antenna);

	_stream = sdr->setupStream( SOAPY_SDR_RX, SOAPY_SDR_CF32);
	if( _stream == nullptr)
	{
    spdlog::error("Failed to set up RX stream");
		SoapySDR::Device::unmake( sdr );
    return false;
	}

  auto sensors = sdr->listSensors();
  if (std::find(sensors.begin(), sensors.end(), "lms7_temp") != sensors.end()) {
    _temp_sensor_available = true;
    _temp_sensor_key = "lms7_temp";
  }

  return true;
}

void SdrReader::start() {
  auto sdr = (SoapySDR::Device*)_sdr;
	sdr->activateStream( (SoapySDR::Stream*)_stream, 0, 0, 0);
  _running = true;

  // Start the reader thread and elevate its priority to realtime
  _readerThread = std::thread{&SdrReader::read, this};
  struct sched_param thread_param = {};
  thread_param.sched_priority = 50;
  _cfg.lookupValue("sdr.reader_thread_priority_rt", thread_param.sched_priority);

  spdlog::debug("Launching sample reader thread with realtime scheduling priority {}", thread_param.sched_priority);

  int error = pthread_setschedparam(_readerThread.native_handle(), SCHED_RR, &thread_param);
  if (error != 0) {
    spdlog::warn("Cannot set reader thread priority to realtime: {}. Thread will run at default priority with a high probability of dropped samples and loss of synchronisation.", strerror(error));
  }
}

void SdrReader::stop() {
  _running = false;

  auto sdr = (SoapySDR::Device*)_sdr;
  sdr->deactivateStream((SoapySDR::Stream*)_stream, 0, 0);

  _readerThread.join();
  _buffer.clear();
}

void SdrReader::read() {
  std::array<void*, SRSLTE_MAX_CHANNELS> radio_buffers = { nullptr };
  while (_running) {
    int toRead = ceil(_sampleRate / 1000.0);
    if (_buffer.free_size() < toRead * sizeof(cf_t)) {
      spdlog::debug("ringbuffer overflow");
      std::this_thread::sleep_for(std::chrono::microseconds(1000));
    } else {
      int read = 0;
      if (_reading_from_file) {
        std::chrono::steady_clock::time_point entered = {};
        entered = std::chrono::steady_clock::now();
        int64_t required_time_us = (1000000.0/_sampleRate) * toRead;

        radio_buffers[0] = _buffer.write_head();
        read = srslte_filesource_read_multi(&file_source, radio_buffers.data(), toRead, 1);
        if ( read == 0 ) {
          srslte_filesource_seek(&file_source, 0);
        }

        if (read> 0) {
          _buffer.commit( read * sizeof(cf_t) );
        }

        std::chrono::microseconds sleep = (std::chrono::microseconds(required_time_us) -
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - entered));
        std::this_thread::sleep_for(sleep);
      } else {
        auto sdr = (SoapySDR::Device*)_sdr;
        int flags;
        long long time_ns;
        void *buffs[] = {_buffer.write_head()}; // NOLINT

        read = sdr->readStream( (SoapySDR::Stream*)_stream, buffs, toRead, flags, time_ns);

        if (read> 0) {
          if (_writing_to_file) {
            srslte_filesink_write(&file_sink, _buffer.write_head(), read);
          }
          _buffer.commit( read * sizeof(cf_t) );
        }
      }
    }
  }
  spdlog::debug("Sample reader thread exited");
}

auto SdrReader::getSamples(cf_t* data, uint32_t nsamples,
                               srslte_timestamp_t *
                               /*rx_time*/) -> int {
  std::chrono::steady_clock::time_point entered = {};
  entered = std::chrono::steady_clock::now();

  int64_t required_time_us = (1000000.0/_sampleRate) * nsamples;
  size_t cnt = nsamples * sizeof(cf_t);

  if (_high_watermark_reached &&  _buffer.size() < (_sampleRate / 1000.0) * 10 * sizeof(cf_t)) {
    _high_watermark_reached = false;
  }

  if (!_high_watermark_reached) {
    spdlog::debug("size {}", _buffer.size() );
    while (_buffer.size() < (_sampleRate / 1000.0) * (_buffer_ms / 2.0) * sizeof(cf_t)) {
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    spdlog::debug("Filled ringbuffer to half capacity");
    _high_watermark_reached = true;
  }

  memcpy(data, _buffer.read_head(), cnt);
  _buffer.consume(cnt);

  if (_buffer.size() < (_sampleRate / 1000.0) * (_buffer_ms / 2.0) * sizeof(cf_t)) {
    required_time_us += 500;
  } else {
    required_time_us -= 500;
  }

  spdlog::debug("read {} samples, adjusted required {} us, delta {} us, sleep adj {},  sleeping for {} us",
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
