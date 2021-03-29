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

#include "LimeSdrReader.h"

#include <chrono>
#include <cmath>

#include "spdlog/spdlog.h"

LimeSdrReader:: ~LimeSdrReader() {
  if (_sdr != nullptr) {
    LMS_StopStream(&_stream);
    LMS_DestroyStream(_sdr, &_stream);
    LMS_Close(_sdr);
  }

  if (_reading_from_file) {
    srslte_filesource_free(&file_source);
  }

  if (_writing_to_file) {
    srslte_filesink_free(&file_sink);
  }
}

auto LimeSdrReader::init(uint8_t device_index, const char* sample_file,
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
    int device_count = LMS_GetDeviceList(nullptr);
    if (device_count < 0) {
      spdlog::error("LMS_GetDeviceList() : {}", LMS_GetLastErrorMessage());
      return false;
    }

    lms_info_str_t device_list[device_count];   // NOLINT
    if (LMS_GetDeviceList(device_list) < 0) {
      spdlog::error("LMS_GetDeviceList() : {}", LMS_GetLastErrorMessage());
      return false;
    }

    if (LMS_Open(&_sdr, device_list[device_index], nullptr) != 0) {
      spdlog::error("LMS_Open() : {}", LMS_GetLastErrorMessage());
      return false;
    }

    if (LMS_Init(_sdr) != 0) {
      spdlog::error("LMS_Init() : {}", LMS_GetLastErrorMessage());
      return false;
    }
    if (LMS_EnableChannel(_sdr, LMS_CH_RX, 0, true) != 0) {
      spdlog::error("LMS_EnableChannel() : {}", LMS_GetLastErrorMessage());
      return false;
    }

    if (LMS_EnableChannel(_sdr, LMS_CH_TX, 0, true) != 0) {
      spdlog::error("LMS_EnableChannel() : {}", LMS_GetLastErrorMessage());
      return false;
    }

    _stream.channel = 0;
    _stream.fifoSize = 1024*1024;
    _stream.throughputVsLatency = 1;
    _stream.isTx = LMS_CH_RX;
    _stream.dataFmt = lms_stream_t::LMS_FMT_F32;
    if (LMS_SetupStream(_sdr, &_stream) != 0) {
      spdlog::error("LMS_SetupStream() : {}", LMS_GetLastErrorMessage());
      return false;
    }

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

void LimeSdrReader::clear_buffer() {
  _buffer.clear();
  _high_watermark_reached = false;
}

auto LimeSdrReader::setSampleRate(unsigned /*sample_rate*/) -> bool {
  return _sdr != nullptr;
}

auto LimeSdrReader::tune(uint32_t frequency, uint32_t sample_rate,
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

  spdlog::info("Tuning Lime SDR Mini to {} MHz, filter bandwidth {} MHz, sample rate {}, gain {}, antenna path {}",
      frequency/1000000.0, bandwidth/1000000.0, sample_rate/1000000.0, gain, antenna);

  if (LMS_SetLOFrequency(_sdr, LMS_CH_RX, 0, frequency) != 0) {
    spdlog::error("LMS_SetLOFrequency() : {}", LMS_GetLastErrorMessage());
    return false;
  }
  double freq = NAN;
  if ( LMS_GetLOFrequency(_sdr, LMS_CH_RX, 0, &freq) != 0 ) {
    spdlog::error("LMS_GetLOFrequency() : {}", LMS_GetLastErrorMessage());
    return false;
  }
  spdlog::info("LO frequency set to {} MHz", freq / 1e6);

  sleep(2);

  int nrAntennas = LMS_GetAntennaList(_sdr, false, 0, nullptr);
  lms_name_t list[nrAntennas];   // NOLINT
  LMS_GetAntennaList(_sdr, false, 0, list);
  int antennaFound = 0;
  for (int i = 0; i < nrAntennas; i++) {
    if (strcmp(list[i], antenna.c_str()) == 0) {
      antennaFound = 1;
      if (LMS_SetAntenna(_sdr, false, 0, i) != 0) {
        spdlog::error("LMS_SetAntenna() : {}", LMS_GetLastErrorMessage());
        return false;
      }
      break;
    }
  }
  if (antennaFound == 0) {
    spdlog::error("Antenna not found: {}", LMS_GetLastErrorMessage());
    return false;
  }

  if (LMS_SetSampleRate(_sdr, sample_rate, 0) != 0) {
    spdlog::error("LMS_SetSampleRate() : {}", LMS_GetLastErrorMessage());
    return false;
  }

  double rf_rate = NAN;
  if (LMS_GetSampleRate(_sdr, LMS_CH_RX, 0, &_sampleRate, &rf_rate) != 0) {
    spdlog::error("LMS_GetSampleRate() : {}", LMS_GetLastErrorMessage());
    return false;
  }
  spdlog::info("Sample rate on host interface {} MHz, RF ADC {} MHz", _sampleRate / 1e6, rf_rate / 1e6);




  if (gain > 0) {
    if (LMS_SetNormalizedGain(_sdr, LMS_CH_RX, 0, gain) != 0) {
      spdlog::error("LMS_SetNormalizedGain() : {}", LMS_GetLastErrorMessage());
      return false;
    }
  }

  float_type setgain = NAN;  // normalized gain
  unsigned int gaindB = 0;   // gain in dB
  if (LMS_GetNormalizedGain(_sdr, LMS_CH_RX, 0, &setgain) != 0) {
    spdlog::error("LMS_GetNr() : {}", LMS_GetLastErrorMessage());
    return false;
  }
  if (LMS_GetGaindB(_sdr, LMS_CH_RX, 0, &gaindB) != 0) {
    spdlog::error("LMS_GetGaindB(() : {}", LMS_GetLastErrorMessage());
    return false;
  }

  spdlog::info("RX gain set to {} / {} dB", setgain, gaindB);

  lms_range_t range;
  if (LMS_GetLPFBWRange(_sdr, LMS_CH_RX, &range) != 0) {
    spdlog::error("LMS_GetLPFBWRange() : {}", LMS_GetLastErrorMessage());
    return false;
  }
  spdlog::debug("LPF BW range {} - {} MHz, step {} Hz", range.min / 1e6, range.max / 1e6, range.step);

  if (LMS_SetLPFBW(_sdr, LMS_CH_RX, 0, bandwidth*1.2) != 0) {
    spdlog::error("LMS_SetLPFBW() : {}", LMS_GetLastErrorMessage());
    return false;
  }

  if (LMS_Calibrate(_sdr, LMS_CH_RX, 0, bandwidth, 0) != 0) {
    spdlog::error("LMS_Calibrate() : {}", LMS_GetLastErrorMessage());
    return false;
  }

  return true;
}

void LimeSdrReader::start() {
  _running = true;
  LMS_StartStream(&_stream);

  // Start the reader thread and elevate its priority to realtime
  _readerThread = std::thread{&LimeSdrReader::read, this};
  struct sched_param thread_param = {};
  thread_param.sched_priority = 50;
  _cfg.lookupValue("sdr.reader_thread_priority_rt", thread_param.sched_priority);

  spdlog::debug("Launching sample reader thread with realtime scheduling priority {}", thread_param.sched_priority);

  int error = pthread_setschedparam(_readerThread.native_handle(), SCHED_RR, &thread_param);
  if (error != 0) {
    spdlog::warn("Cannot set reader thread priority to realtime: {}. Thread will run at default priority with a high probability of dropped samples and loss of synchronisation.", strerror(error));
  }
}

void LimeSdrReader::stop() {
  _running = false;
  LMS_StopStream(&_stream);

  _readerThread.join();
  _buffer.clear();
}

void LimeSdrReader::read() {
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
        read = LMS_RecvStream(&_stream, _buffer.write_head(), toRead, nullptr,
                              1000);
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

auto LimeSdrReader::getSamples(cf_t* data, uint32_t nsamples,
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
