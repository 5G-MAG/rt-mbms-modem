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

//#include "RestHandler.h"
#include "CasFrameProcessor.h"

#include <memory>
#include <utility>

#include "spdlog/spdlog.h"

using web::json::value;
using web::http::methods;
using web::http::uri;
using web::http::http_request;
using web::http::status_codes;
using web::http::experimental::listener::http_listener;
using web::http::experimental::listener::http_listener_config;

RestHandler::RestHandler(const libconfig::Config& cfg, const std::string& url,
                         state_t& state, SdrReader& sdr, Phy& phy,
                         set_params_t set_params)
    : _cfg(cfg),
      _state(state),
      _sdr(sdr),
      _phy(phy),
      _set_params(std::move(set_params)) {

  http_listener_config server_config;
  if (url.rfind("https", 0) == 0) {
    server_config.set_ssl_context_callback(
        [&](boost::asio::ssl::context& ctx) {
          std::string cert_file = "/usr/share/5gmag-rt/cert.pem";
          cfg.lookupValue("modem.restful_api.cert", cert_file);

          std::string key_file = "/usr/share/5gmag-rt/key.pem";
          cfg.lookupValue("modem.restful_api.key", key_file);

          ctx.set_options(boost::asio::ssl::context::default_workarounds);
          ctx.use_certificate_chain_file(cert_file);
          ctx.use_private_key_file(key_file, boost::asio::ssl::context::pem);
        });
  }

  cfg.lookupValue("modem.restful_api.api_key.enabled", _require_bearer_token);
  if (_require_bearer_token) {
    _api_key = "106cd60-76c8-4c37-944c-df21aa690c1e";
    cfg.lookupValue("modem.restful_api.api_key.key", _api_key);
  }

  _listener = std::make_unique<http_listener>(
      url, server_config);

  _listener->support(methods::GET, std::bind(&RestHandler::get, this, std::placeholders::_1));  // NOLINT
  _listener->support(methods::PUT, std::bind(&RestHandler::put, this, std::placeholders::_1));  // NOLINT

  //_listener->open().wait();
}

RestHandler::~RestHandler() = default;

void RestHandler::get(http_request message) {
  spdlog::debug("Received GET request {}", message.to_string() );
  auto paths = uri::split_path(uri::decode(message.relative_uri().path()));
  if (_require_bearer_token &&
    (message.headers()["Authorization"] != "Bearer " + _api_key)) {
    message.reply(status_codes::Unauthorized);
    return;
  }

  if (paths.empty()) {
    message.reply(status_codes::NotFound);
  } else {
    if (paths[0] == "status") {
      auto state = value::object();

      switch (_state) {
        case searching:
          state["state"] = value::string("searching");
          break;
        case syncing:
          state["state"] = value::string("syncing");
          break;
        case processing:
          state["state"] = value::string("synchronized");
          break;
      }

      if (_phy.cell().nof_prb == _phy.cell().mbsfn_prb) {
        state["nof_prb"] = value(_phy.cell().nof_prb);
      } else {
        state["nof_prb"] = value(_phy.cell().mbsfn_prb);
      }
      state["cell_id"] = value(_phy.cell().id);
      state["cfo"] = value(_phy.cfo());
      state["cinr_db"] = value(cinr_db());
      state["cinr_db_avg"] = value(cinr_db_avg());
      state["subcarrier_spacing"] = value(_phy.mbsfn_subcarrier_spacing_khz());

      // CAS Chest params //
      state["filter_order"] = value(_cas_processor->get_filter_order());
      state["filter_coef"] = value(_cas_processor->get_filter_coef());
      state["filter_type"] = value(_cas_processor->get_filter_type());
      state["noise_alg"] = value(_cas_processor->get_noise_alg());
      state["sync_error"] = value(_cas_processor->get_sync_error());
      state["estimator_alg"] = value(_cas_processor->get_estimator_alg());
      state["cfo_estimate"] = value(_cas_processor->get_cfo_estimate());
      state["evm_meas"] = value(_cas_processor->get_evm_meas());
      
      // Phy params // 
      state["cfo_est_pss_find"] = value(_phy.get_ue_sync_find_cfo_pss_enable());
      state["cfo_est_pss_track"] = value(_phy.get_ue_sync_track_cfo_pss_enable());
      state["cfo_correct_find"] = value(_phy.get_ue_sync_find_cfo_correct_enable());
      state["cfo_correct_track"] = value(_phy.get_ue_sync_track_cfo_correct_enable());
      state["cfo_pss_loop_bw"] = value(_phy.get_ue_sync_cfo_loop_bw_pss());
      state["cfo_ema_alpha_find"] = value(_phy.get_ue_sync_find_cfo_ema());
      state["cfo_ema_alpha_track"] = value(_phy.get_ue_sync_track_cfo_ema());
      state["pss_ema_find"] = value(_phy.get_ue_sync_pss_cfo_ema_find());
      state["pss_ema_track"] = value(_phy.get_ue_sync_pss_cfo_ema_track());
      state["threshold_find"] = value(_phy.get_ue_sync_threshold_find());
      state["threshold_track"] = value(_phy.get_ue_sync_threshold_track());


      message.reply(status_codes::OK, state);
    } else if (paths[0] == "sdr_params") {
      value sdr = value::object();
      sdr["frequency"] = value(_sdr.get_frequency());
      sdr["gain"] = value(_sdr.get_gain());
      sdr["min_gain"] = value(_sdr.min_gain());
      sdr["max_gain"] = value(_sdr.max_gain());
      sdr["filter_bw"] = value(_sdr.get_filter_bw());
      sdr["antenna"] = value(_sdr.get_antenna());
      sdr["sample_rate"] = value(_sdr.get_sample_rate());
      sdr["buffer_level"] = value(_sdr.get_buffer_level());
      message.reply(status_codes::OK, sdr);
    } else if (paths[0] == "ce_values") {
      auto cestream = Concurrency::streams::bytestream::open_istream(_ce_values);
      message.reply(status_codes::OK, cestream);
    } else if (paths[0] == "cir_values") {
      auto cestream = Concurrency::streams::bytestream::open_istream(_cir_values);
      message.reply(status_codes::OK, cestream);
    } else if (paths[0] == "cir_values_mbsfn") {
      auto cestream = Concurrency::streams::bytestream::open_istream(_cir_values_mbsfn);
      message.reply(status_codes::OK, cestream);
    } else if (paths[0] == "corr_values") {
      auto cestream = Concurrency::streams::bytestream::open_istream(_corr_values);
      message.reply(status_codes::OK, cestream);
    } else if (paths[0] == "corr_values_mbsfn") {
      auto cestream = Concurrency::streams::bytestream::open_istream(_corr_values_mbsfn);
      message.reply(status_codes::OK, cestream);
    } else if (paths[0] == "pdsch_status") {
      value sdr = value::object();
      sdr["bler"] = value(static_cast<float>(_pdsch.errors) /
                                static_cast<float>(_pdsch.total));
      sdr["ber"] = value(_pdsch.ber);
      sdr["mcs"] = value(_pdsch.mcs);
      sdr["present"] = 1;
      message.reply(status_codes::OK, sdr);
    } else if (paths[0] == "pdsch_data") {
      auto cestream = Concurrency::streams::bytestream::open_istream(_pdsch.GetData());
      message.reply(status_codes::OK, cestream);
    } else if (paths[0] == "mcch_status") {
      value sdr = value::object();
      sdr["bler"] = value(static_cast<float>(_mcch.errors) /
                                static_cast<float>(_mcch.total));
      sdr["ber"] = value(_mcch.ber);
      sdr["mcs"] = value(_mcch.mcs);
      sdr["present"] = 1;
      message.reply(status_codes::OK, sdr);
    } else if (paths[0] == "mcch_data") {
      auto cestream = Concurrency::streams::bytestream::open_istream(_mcch.GetData());
      message.reply(status_codes::OK, cestream);
    } else if (paths[0] == "mch_info") {
      std::vector<value> mi;
      auto mch_info = _phy.mch_info();
      std::for_each(std::begin(mch_info), std::end(mch_info), [&mi](Phy::mch_info_t const& mch) {
          value m;
          m["mcs"] = value(mch.mcs);
          std::vector<value> mti;
          std::for_each(std::begin(mch.mtchs), std::end(mch.mtchs), [&mti](Phy::mtch_info_t const& mtch) {
              value mt;
              mt["tmgi"] = value(mtch.tmgi);
              mt["dest"] = value(mtch.dest);
              mt["lcid"] = value(mtch.lcid);
              mti.push_back(mt);
          });
          m["mtchs"] = value::array(mti);
          mi.push_back(m);
      });
      message.reply(status_codes::OK, value::array(mi));
    } else if (paths[0] == "mch_status") {
      int idx = std::stoi(paths[1]);
      value sdr = value::object();
      sdr["bler"] = value(static_cast<float>(_mch[idx].errors) /
                                static_cast<float>(_mch[idx].total));
      sdr["ber"] = value(_mch[idx].ber);
      sdr["mcs"] = value(_mch[idx].mcs);
      sdr["present"] = value(_mch[idx].present);
      message.reply(status_codes::OK, sdr);
    } else if (paths[0] == "mch_data") {
      int idx = std::stoi(paths[1]);
      auto cestream = Concurrency::streams::bytestream::open_istream(_mch[idx].GetData());
      message.reply(status_codes::OK, cestream);
    } else if (paths[0] == "log") {
      std::string logfile = "/var/log/syslog";

      Concurrency::streams::file_stream<uint8_t>::open_istream(logfile).then(
          [message](const Concurrency::streams::basic_istream<unsigned char>&
                        file_stream) {
            message.reply(status_codes::OK, file_stream, "text/plain");
          });
    }
  }
}

void RestHandler::put(http_request message) {
  spdlog::debug("Received PUT request {}", message.to_string() );

  if (_require_bearer_token &&
    (message.headers()["Authorization"] != "Bearer " + _api_key)) {
    message.reply(status_codes::Unauthorized);
    return;
  }

  auto paths = uri::split_path(uri::decode(message.relative_uri().path()));
  if (paths.empty()) {
    message.reply(status_codes::NotFound);
  } else {
    if (paths[0] == "sdr_params") {
      value answer;

      auto f = _sdr.get_frequency();
      auto g = _sdr.get_gain();
      auto bw = _sdr.get_filter_bw();
      auto a = _sdr.get_antenna();
      auto sr = _sdr.get_sample_rate();

      const auto & jval = message.extract_json().get();
      spdlog::debug("Received JSON: {}", jval.serialize());

      if (jval.has_field("antenna")) {
        a = jval.at("antenna").as_string();
      }
      if (jval.has_field("frequency")) {
        f = jval.at("frequency").as_integer();
      }
      if (jval.has_field("gain")) {
        g = jval.at("gain").as_double();
      }
      _set_params( a, f, g, sr, bw);

      message.reply(status_codes::OK, answer);
    } else if (paths[0] == "chest_cfg_params") {
      value answer;

      const auto & jval = message.extract_json().get();
      spdlog::debug("Recieved JSON: {}", jval.serialize());

      if (jval.has_field("noise_alg")) {
        auto alg = jval.at("noise_alg").as_string();
        spdlog::info("New alg est {}", alg);
        _cas_processor->set_noise_alg(static_cast<srsran_chest_dl_noise_alg_t>(stoi(alg)));
      }
      if (jval.has_field("sync_error")) {
        spdlog::info("New sync error value");
        
        bool alg = jval.at("sync_error").as_bool();

        spdlog::info("{}", alg); 
        _cas_processor->set_sync_error(alg);
      }
      if (jval.has_field("estimator_alg")) {
        auto alg = jval.at("estimator_alg").as_string();
        spdlog::info("New alg est {}", alg);
        _cas_processor->set_estimator_alg(static_cast<srsran_chest_dl_estimator_alg_t>(stoi(alg)));
      }
      if (jval.has_field("filter_type")) {
        auto type = jval.at("filter_type").as_string();
        spdlog::info("New filter type {}", type);
        _cas_processor->set_filter_type(static_cast<srsran_chest_filter_t>(stoi(type)));
      }
      if (jval.has_field("filter_order")) {
        spdlog::info("New filter order");
        auto order = jval.at("filter_order").as_integer();
        _cas_processor->set_filter_order(order);
      }
      if (jval.has_field("filter_coef")) {
        spdlog::info("New filter coef");
        auto coef = jval.at("filter_coef").as_double();
        _cas_processor->set_filter_coef(coef);
      }

      // Phy params
      if (jval.has_field("cfo_est_pss_find")) {
        spdlog::info("New cfo est pss find");
        auto toggle = jval.at("cfo_est_pss_find").as_bool();
        _phy.set_ue_sync_find_cfo_pss_enable(toggle);
      }
      if (jval.has_field("cfo_est_pss_track")) {
        spdlog::info("New cfo est pss track");
        auto toggle = jval.at("cfo_est_pss_track").as_bool();
        _phy.set_ue_sync_track_cfo_pss_enable(toggle);
      }
      if (jval.has_field("cfo_correct_find")) {
        spdlog::info("New cfo correct find");
        auto toggle = jval.at("cfo_correct_find").as_bool();
        _phy.set_ue_sync_find_cfo_correct_enable(toggle);
      }
      if (jval.has_field("cfo_correct_track")) {
        spdlog::info("New cfo correct track");
        auto toggle = jval.at("cfo_correct_track").as_bool();
        _phy.set_ue_sync_track_cfo_correct_enable(toggle);
      }
      if (jval.has_field("cfo_pss_loop_bw")) {
        spdlog::info("New BW pss CFO loop");
        auto bw = jval.at("cfo_pss_loop_bw").as_double();
        _phy.set_ue_sync_cfo_loop_bw_pss(bw);
      }
      if (jval.has_field("cfo_ema_alpha_find")) {
        spdlog::info("New CFO ema alpha for find");
        auto ema = jval.at("cfo_ema_alpha_find").as_double();
        _phy.set_ue_sync_find_cfo_ema(ema);
      }
      if (jval.has_field("cfo_ema_alpha_track")) {
        spdlog::info("New CFO ema alpha for track");
        auto ema = jval.at("cfo_ema_alpha_track").as_double();
        _phy.set_ue_sync_track_cfo_ema(ema); 
      }
      if (jval.has_field("pss_ema_find")) {
        spdlog::info("New PSS corr  ema alpha for find");
        auto ema = jval.at("pss_ema_find").as_double();
        _phy.set_ue_sync_pss_cfo_ema_find(ema); 
      }
      if (jval.has_field("pss_ema_track")) {
        spdlog::info("New PSS corr  ema alpha for track");
        auto ema = jval.at("pss_ema_track").as_double();
        _phy.set_ue_sync_pss_cfo_ema_track(ema); 
      }
      if (jval.has_field("threshold_find")) {
        spdlog::info("New threshold for find");
        auto ema = jval.at("threshold_find").as_double();
        _phy.set_ue_sync_threshold_find(ema); 
      }
      if (jval.has_field("threshold_track")) {
        spdlog::info("New threshold for track");
        auto ema = jval.at("threshold_track").as_double();
        _phy.set_ue_sync_threshold_track(ema); 
      }
      
      message.reply(status_codes::OK, answer);
    }
  }
}

void RestHandler::add_cinr_value( float cinr) {
  if (_cinr_db.size() > CINR_RAVG_CNT) {
    _cinr_db.erase(_cinr_db.begin());
  }
  _cinr_db.push_back(cinr);
}
