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
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include "MeasurementFileWriter.h"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <numeric>
#include <vector>

#include "spdlog/spdlog.h"

const uint32_t kGpsWaitTimeout = 5000000;
const uint32_t kGpsReconnectTimeout = 5000;
const uint32_t kGpsWaitMicrosleep = 100;
const uint32_t kMaxTimestringSize = 80;

MeasurementFileWriter::MeasurementFileWriter(const libconfig::Config& cfg)
      : _cfg(cfg) {
  bool gps_enabled = false;
  _cfg.lookupValue("modem.measurement_file.gpsd.enabled", gps_enabled);

  if (gps_enabled) {
    std::string host = "localhost";
    _cfg.lookupValue("modem.measurement_file.gpsd.host", host);

    std::string port = DEFAULT_GPSD_PORT;
    _cfg.lookupValue("modem.measurement_file.gpsd.port", port);

    _gps = std::make_unique<gpsmm>(host.c_str(), port.c_str());

    if (_gps->stream(WATCH_ENABLE | WATCH_JSON) == nullptr) {
      spdlog::error("No GPSD running, cannot start GPS stream.");
    } else {
      spdlog::info("GPS data stream started");
    }

    _gps_reader_thread = std::thread{&MeasurementFileWriter::ReadGps, this};
  }
}

MeasurementFileWriter::~MeasurementFileWriter() {
  _running = false;
  _gps_reader_thread.join();
}

void MeasurementFileWriter::ReadGps() {
  while (_running) {
    if (_gps->is_open()) {
      if (!_gps->waiting(kGpsWaitTimeout)) {
        {
          continue;
        }
      }

      if ((_gps_data = _gps->read()) != nullptr) {
        if ((_gps_data->set & TIME_SET) != 0) {
          struct tm ts = *localtime(&_gps_data->fix.time.tv_sec);
          std::string buf;
          buf.resize(kMaxTimestringSize);
          strftime(&buf[0], buf.size(), "%FT%T", &ts);
          _last_gps_time = buf;
        }
        if ((_gps_data->set & LATLON_SET) != 0) {
          _last_gps_lat = std::to_string(_gps_data->fix.latitude);
          _last_gps_lng = std::to_string(_gps_data->fix.longitude);
          _last_gps_speed = std::to_string(_gps_data->fix.speed);
        }
      } else {
        _last_gps_speed = _last_gps_lat = _last_gps_lng = _last_gps_time = "";
      }

      std::this_thread::sleep_for(std::chrono::microseconds(kGpsWaitMicrosleep));
    } else {
      _last_gps_speed = _last_gps_lat = _last_gps_lng = _last_gps_time = "";
      std::this_thread::sleep_for(std::chrono::microseconds(kGpsReconnectTimeout));
      _gps->stream(WATCH_ENABLE|WATCH_JSON);
    }
  }
}

void MeasurementFileWriter::WriteLogValues(
    const std::vector<std::string>& values) {
  time_t now = time(nullptr);
  struct tm ts = *localtime(&now);
  std::string buf;
  buf.resize(kMaxTimestringSize);
  strftime(&buf[0], buf.size(), "%FT%T", &ts);

  std::vector<std::string> cols = {buf, _last_gps_lat,
    _last_gps_lng, _last_gps_speed, _last_gps_time};

  std::string line;
  for (const auto& col : cols) {
    {
      line += col + ";";
    }
  }
  for (const auto& val : values) {
    {
      line += val + ";";
    }
  }

  std::string file_loc = "/tmp/modem_measurements.csv";
  _cfg.lookupValue("modem.measurement_file.file_path", file_loc);
  std::ofstream file;
  file.open(file_loc, std::ios_base::app);
  file << line << std::endl;
  file.close();
}
