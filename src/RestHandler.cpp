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

#include "RestHandler.h"

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
                         state_t& state, LimeSdrReader& lime, Phy& phy,
                         set_params_t set_params)
    : _cfg(cfg),
      _state(state),
      _lime(lime),
      _phy(phy),
      _set_params(std::move(set_params)) {

  http_listener_config server_config;
  if (url.rfind("https", 0) == 0) {
    server_config.set_ssl_context_callback(
        [&](boost::asio::ssl::context& ctx) {
          std::string cert_file = "/usr/share/obeca/cert.pem";
          cfg.lookupValue("restful_api.cert", cert_file);

          std::string key_file = "/usr/share/obeca/key.pem";
          cfg.lookupValue("restful_api.key", key_file);

          ctx.set_options(boost::asio::ssl::context::default_workarounds);
          ctx.use_certificate_chain_file(cert_file);
          ctx.use_private_key_file(key_file, boost::asio::ssl::context::pem);
        });
  }

  cfg.lookupValue("restful_api.api_key.enabled", _require_bearer_token);
  if (_require_bearer_token) {
    _api_key = "106cd60-76c8-4c37-944c-df21aa690c1e";
    cfg.lookupValue("restful_api.api_key.key", _api_key);
  }

  _listener = std::make_unique<http_listener>(
      url, server_config);

  _listener->support(methods::GET, std::bind(&RestHandler::get, this, std::placeholders::_1));  // NOLINT
  _listener->support(methods::PUT, std::bind(&RestHandler::put, this, std::placeholders::_1));  // NOLINT

  _listener->open().wait();
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

      state["nof_prb"] = value(_phy.cell().nof_prb);
      state["cell_id"] = value(_phy.cell().id);
      state["cfo"] = value(_phy.cfo());
      state["subcarrier_spacing"] = value(_phy.mbsfn_subcarrier_spacing_khz());
      message.reply(status_codes::OK, state);
    } else if (paths[0] == "sdr_params") {
      value sdr = value::object();
      sdr["frequency"] = value(_lime.get_frequency());
      sdr["gain"] = value(_lime.get_gain());
      sdr["filter_bw"] = value(_lime.get_filter_bw());
      sdr["antenna"] = value(_lime.get_antenna());
      sdr["sample_rate"] = value(_lime.get_sample_rate());
      sdr["buffer_level"] = value(_lime.get_buffer_level());
      message.reply(status_codes::OK, sdr);
    } else if (paths[0] == "ce_values") {
      auto cestream = Concurrency::streams::bytestream::open_istream(_ce_values);
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
      sdr["ber"] = value("-");
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
      sdr["ber"] = value("-");
      sdr["mcs"] = value(_mch[idx].mcs);
      sdr["present"] = value(_mch[idx].present);
      message.reply(status_codes::OK, sdr);
    } else if (paths[0] == "mch_data") {
      int idx = std::stoi(paths[1]);
      auto cestream = Concurrency::streams::bytestream::open_istream(_mch[idx].GetData());
      message.reply(status_codes::OK, cestream);
    } else if (paths[0] == "log") {
      std::string logfile = "/tmp/ofr.log";
      _cfg.lookupValue("app.log_file", logfile);

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

      const auto & jval = message.extract_json().get();

      _set_params(
          jval.at("antenna").as_string(),
          jval.at("frequency").as_integer(),
          jval.at("gain").as_double(),
          jval.at("sample_rate").as_integer(),
          jval.at("filter_bw").as_integer());

      message.reply(status_codes::OK, answer);
    }
  }
}
