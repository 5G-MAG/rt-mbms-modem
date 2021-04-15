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
// GNU Affero General Public License for more details.
// 
// You should have received a copy of the GNU Affero General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#pragma once


#include <ring_buffer.h>
#include <string>
#include <thread>
#include <cstdint>
#include <libconfig.h++>
#include "srslte/srslte.h"

/**
 *  Interface to the SDR stick.
 *
 *  Sets up the SDR, reads samples from it, and handles a ringbuffer for 
 *  the received samples.
 */
class SdrReader {
 public:
    /**
     *  Default constructor.
     *
     *  @param cfg Config singleton reference
     */
    explicit SdrReader(const libconfig::Config& cfg)
      : _buffer(bev::linear_ringbuffer::delayed_init {})
      , _overflows(0)
      , _underflows(0)
      , _cfg(cfg)
      , _readerThread{} {}

    /**
     *  Default destructor.
     */
    virtual ~SdrReader();

    /**
     * Prints a list of all available SDR devices
     */
    void enumerateDevices();

    /**
     * Initializes the SDR interface and creates a ring buffer according to the params from Cfg.
     */
    bool init(const std::string& device_args, const char* sample_file, const char* write_sample_file);

    /**
     * Adjust the sample rate
     */
    bool setSampleRate(unsigned sample_rate);

    /**
     * Tune the SDR to the desired frequency, and set gain, filter and antenna parameters.
     */
    bool tune(uint32_t frequency, uint32_t sample_rate, uint32_t bandwidth, double gain, const std::string& antenna);

    /**
     * Start reading samples from the SDR
     */
    void start();

    /**
     * Stop reading samples from the SDR
     */
    void stop();

    /**
     * Clear all samples from the rx buffers
     */
    void clear_buffer();

    /**
     * Store nsamples count samples into the buffer at data
     *
     * @param data Buffer pointer
     * @param nsamples sample count
     * @param rx_time unused
     */
    int getSamples(cf_t* data, uint32_t nsamples, srslte_timestamp_t* rx_time);

    /**
     * Get current sample rate
     */
    double get_sample_rate() { return _sampleRate; }

    /**
     * Get current center frequency
     */
    double get_frequency() { return _frequency; }

    /**
     * Get current filter bandwidth
     */
    unsigned get_filter_bw() { return _filterBw; }

    /**
     * Get current gain
     */
    double get_gain() { return _gain; }

    /**
     * Get current ringbuffer level (0 = empty .. 1 = full)
     */
    double get_buffer_level() { if (!_buffer_ready) { return 0; } return static_cast<double>(_buffer.size()) / static_cast<double>(_buffer.capacity()); }

    /**
     * Get current antenna port
     */
    std::string get_antenna() { return _antenna; }

    /**
     * Get RSSI estimate (disabled at the moment)
     */
    uint32_t rssi() { return _rssi; }

 private:
    void read();
    void* _sdr = nullptr;
    void* _stream = nullptr;

    const libconfig::Config& _cfg;
    bev::linear_ringbuffer _buffer;
    std::thread _readerThread;
    bool _running;

    double _sampleRate;
    double _frequency;
    unsigned _filterBw;
    double _gain;
    std::string _antenna;
    unsigned _overflows;
    unsigned _underflows;

    cf_t* _read_buffer;

    srslte_filesource_t          file_source;
    srslte_filesink_t          file_sink;

    std::chrono::steady_clock::time_point _last_read;

    bool _high_watermark_reached = false;
    int _sleep_adjustment = 0;

    unsigned _buffer_ms = 200;
    bool _buffer_ready = false;
    bool _reading_from_file = false;
    bool _writing_to_file = false;

    uint32_t _rssi = 0;

    bool _temp_sensor_available = false;
    std::string _temp_sensor_key = {};
};
