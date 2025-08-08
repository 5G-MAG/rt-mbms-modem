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

#include <string>
#include <vector>
#include <thread>
#include <map>
#include <cstdint>
#include <libconfig.h++>
#include "srsran/srsran.h"
#include "MultichannelRingbuffer.h"

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
    explicit SdrReader(const libconfig::Config &cfg, size_t rx_channels)
            : _overflows(0), _underflows(0), _cfg(cfg), _rx_channels(rx_channels), _readerThread{} {}

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
    bool init(const std::string &device_args, const char *sample_file, const char *write_sample_file, bool repeat_sample_file);

    /**
     * Tune the SDR to the desired frequency, and set gain, filter and antenna parameters.
     */
    bool tune(uint32_t frequency, uint32_t sample_rate, uint32_t bandwidth, double gain, const std::string &antenna,
              bool use_agc);

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
    int get_samples(cf_t *data[SRSRAN_MAX_CHANNELS], uint32_t nsamples, srsran_timestamp_t *rx_time);

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
    double get_buffer_level();

    /**
     * Get current antenna port
     */
    std::string get_antenna() { return _antenna; }

    /**
     * Get RSSI estimate (disabled at the moment)
     */
    uint32_t rssi() { return _rssi; }

    double min_gain() { return _min_gain; }

    double max_gain() { return _max_gain; }

    /**
     * If sample file creation is enabled, writing samples starts after this call
     */
    void enableSampleFileWriting() { _write_samples = true; }

    /**
     * If sample file creation is enabled, writing samples stops after this call
     */
    void disableSampleFileWriting() { _write_samples = false; }

private:
    void init_buffer();

    bool set_gain(bool use_agc, double gain, uint8_t idx);

    bool set_sample_rate(uint32_t rate, uint8_t idx);

    bool set_filter_bw(uint32_t bandwidth, uint8_t idx);

    bool set_antenna(const std::string &antenna, uint8_t idx);

    bool set_frequency(uint32_t frequency, uint8_t idx);

    void read();

    void *_sdr = nullptr;
    void *_stream = nullptr;

    const libconfig::Config &_cfg;

    std::unique_ptr<MultichannelRingbuffer> _buffer;
    std::unique_ptr<MultichannelRingbuffer> _buffer_write;

    std::thread _readerThread;
    bool _running;

    unsigned _rx_channels = 1;
    double _sampleRate;
    double _frequency;
    unsigned _filterBw;
    double _gain;
    bool _use_agc;
    double _min_gain;
    double _max_gain;
    std::string _antenna;
    unsigned _overflows;
    unsigned _underflows;

    cf_t *_read_buffer;

    srsran_filesource_t file_source;
    srsran_filesink_t file_sink;

    std::chrono::steady_clock::time_point _last_read;

    bool _high_watermark_reached = false;
    int _sleep_adjustment = 0;

    unsigned _buffer_ms = 200;
    bool _buffer_ready = false;
    bool _reading_from_file = false;
    bool _writing_to_file = false;
    bool _write_samples = false;
    bool _repeat_sample_file = false;

    uint32_t _rssi = 0;

    bool _temp_sensor_available = false;
    std::string _temp_sensor_key = {};

    std::map<std::string, std::string> _device_args;
};
