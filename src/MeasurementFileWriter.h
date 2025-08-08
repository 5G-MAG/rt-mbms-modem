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

#pragma once

#include <libgpsmm.h>

#include <string>
#include <thread>
#include <cstdint>
#include <memory>
#include <vector>

#include <libconfig.h++>

/**
 *  Writes measurement data / current reception parameters to a file
 */
class MeasurementFileWriter {
 public:
    /**
     *  Default constructor.
     *
     *  @param cfg Config singleton reference
     */
    explicit MeasurementFileWriter(const libconfig::Config& cfg);

    /**
     *  Default destructor.
     */
    virtual ~MeasurementFileWriter();

    /**
     *  Write a line containing the passed values.
     *
     *  This methods always writes the first four columns containing:
     *  - the current timestamp
     *  - the GPS latitude (if a GPS device was configured)
     *  - the GPS longitude (if a GPS device was configured)
     *  - the GPS time (if a GPS device was configured)
     *
     *  followed by the values in the values vector.
     */
    void WriteLogValues(const std::vector<std::string>& values);

 private:
    void ReadGps();

    const libconfig::Config& _cfg;

    std::unique_ptr<gpsmm> _gps;
    struct gps_data_t* _gps_data {};
    std::thread _gps_reader_thread;
    bool _running = true;

    std::string _last_gps_speed = "";
    std::string _last_gps_lat = "";
    std::string _last_gps_lng = "";
    std::string _last_gps_time = "";
};
