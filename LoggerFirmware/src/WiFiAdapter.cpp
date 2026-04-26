/*! \file WiFiAdapter.cpp
 *  \brief Interface for the WiFi adapter on the module, and a factory to abstract the object
 */

#include <functional>
#include <queue>
#include <vector>

#include <WiFi.h>
#include <WiFiAP.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WifiClient.h>
#include <LittleFS.h>
#include <ESP32-targz.h>
#include "esp_wifi.h"

#include "LogManager.h"
#include "WiFiAdapter.h"
#include "Configuration.h"
#include "MemController.h"
#include "serial_number.h"

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)

class ChunkedStream : public Stream {
public:
    ChunkedStream(WebServer *output) : m_output(output) {}
    int available() override { return m_output->client().available(); }
    int read() override { return m_output->client().read(); }
    int peek() override { return m_output->client().peek(); }
    size_t write(uint8_t b) override { return m_output->client().write(b); }
    size_t write(const uint8_t *buf, size_t size) override {
        m_output->sendContent((const char *)buf, size);
        return size;
    }
private:
    WebServer *m_output;
};

class ExtendedWebServer : public WebServer {
public:
    ExtendedWebServer(int port = 80) : WebServer(port) {}
    size_t StreamArchive(fs::FS *source, const char *path) {
        TAR::dir_entities_t dirEntries;
        TAR::collectDirEntities(&dirEntries, source, path);
        if (dirEntries.size() == 0) return 0;
        setContentLength(CONTENT_LENGTH_UNKNOWN);
        String headers, module_id;
        if (!logger::LoggerConfig.GetConfigString(logger::Config::ConfigParam::CONFIG_MODULEID_S, module_id))
            module_id = "wibl-logs";
        if (module_id.startsWith("TNODEID")) module_id = "wibl-logs";
        module_id += ".tgz";
        sendHeader("Content-Disposition", String("attachment; filename=\"") + module_id + "\"");
        _prepareHeader(headers, 200, "application/tar+gzip", CONTENT_LENGTH_UNKNOWN);
        _currentClient.write(headers.c_str(), headers.length());
        ChunkedStream op(this);
        return TarGzPacker::compress(source, dirEntries, &op);
    }
};

class ConnectionStateMachine {
public:
    ConnectionStateMachine(bool verbose = false) : m_verbose(verbose) {
        m_lastConnectAttempt = m_lastStatusCheck = millis();
        m_connectionRetries = maximumReties();
        m_retryDelay = retryDelay();
        m_connectDelay = connectionDelay();
        m_statusDelay = 500;
        m_lastScanTime = 0;
        m_scanStarted = false;
        m_currentState = STOPPED;
    }

    bool Verbose(void) { return m_verbose; }
    void Verbose(bool verbose) { m_verbose = verbose; }

    void Start(void) {
        if (WiFiAdapter::GetWirelessMode() == WiFiAdapter::WirelessMode::ADAPTER_SOFTAP) {
            m_currentState = AP_MODE;
            logger::LoggerConfig.SetConfigString(logger::Config::ConfigParam::CONFIG_WS_STATUS_S, "AP-Enabled");
            apSetup();
        } else {
            logger::LoggerConfig.SetConfigString(logger::Config::ConfigParam::CONFIG_WS_STATUS_S, "Station-Enabled,Connecting");
            m_currentState = STATION_CONNECTING;
            if (attemptStationJoin()) m_currentState = STATION_CONNECTED;
        }
    }

    void StepState(void) {
        using namespace logger;
        int now = millis();
        switch (m_currentState) {
            case STOPPED: break;
            case AP_MODE:
                if (WiFiAdapter::GetWirelessMode() == WiFiAdapter::WirelessMode::ADAPTER_STATION) {
                    if (!m_scanStarted) {
                        String scan_interval_s;
                        long scan_interval_ms = 300000;
                        if (LoggerConfig.GetConfigString(Config::ConfigParam::CONFIG_STATION_SCAN_INTERVAL_S, scan_interval_s))
                            scan_interval_ms = scan_interval_s.toInt() * 1000;
                        if (m_lastScanTime == 0 || (now - m_lastScanTime) > scan_interval_ms) {
                            if (m_verbose) Serial.printf("DBG: triggering background scan (interval %ld ms)...\n", scan_interval_ms);
                            WiFi.scanNetworks(true);
                            m_scanStarted = true;
                            m_lastScanTime = now;
                        }
                    } else {
                        int16_t n = WiFi.scanComplete();
                        if (n >= 0) {
                            String targetSsid;
                            LoggerConfig.GetConfigString(Config::ConfigParam::CONFIG_STATION_SSID_S, targetSsid);
                            bool found = false;
                            for (int i = 0; i < n; ++i) if (WiFi.SSID(i) == targetSsid) { found = true; break; }
                            if (found) {
                                if (m_verbose) Serial.printf("DBG: found hotspot %s, reconnecting...\n", targetSsid.c_str());
                                WiFi.scanDelete(); m_scanStarted = false;
                                if (attemptStationJoin()) m_currentState = STATION_CONNECTED;
                                else m_currentState = STATION_CONNECTING;
                            } else { WiFi.scanDelete(); m_scanStarted = false; }
                        } else if (n == WIFI_SCAN_FAILED) m_scanStarted = false;
                    }
                }
                break;
            case STATION_CONNECTING:
                if ((now - m_lastStatusCheck) > m_statusDelay) {
                    if (isConnected()) m_currentState = STATION_CONNECTED;
                    else if ((now - m_lastConnectAttempt) > m_connectDelay) {
                        m_currentState = STATION_RETRY;
                        LoggerConfig.SetConfigString(Config::CONFIG_WS_STATUS_S, "Station-Enabled,Connect-Timeout-Retrying");
                    }
                }
                break;
            case STATION_RETRY:
                if ((now - m_lastConnectAttempt) > m_retryDelay) {
                    if (m_connectionRetries > 0) {
                        --m_connectionRetries;
                        if (attemptStationJoin()) m_currentState = STATION_CONNECTED;
                        else m_currentState = STATION_CONNECTING;
                    } else {
                        m_currentState = MOVE_TO_SAFE_MODE;
                    }
                }
                break;
            case MOVE_TO_SAFE_MODE:
                LoggerConfig.SetConfigString(Config::ConfigParam::CONFIG_WS_STATUS_S, "AP-Fallback,Station-Join-Failed");
                apSetup();
                m_currentState = AP_MODE;
                m_lastScanTime = now;
                break;
            case STATION_CONNECTED:
                logger::LoggerConfig.SetConfigString(logger::Config::ConfigParam::CONFIG_WS_STATUS_S, "Station-Enabled,Connected");
                m_currentState = CONNECTION_CHECK;
                completeStationJoin();
                break;
            case CONNECTION_CHECK:
                if ((now - m_lastStatusCheck) > m_retryDelay) {
                    if (!isConnected()) {
                        logger::LoggerConfig.SetConfigString(logger::Config::ConfigParam::CONFIG_WS_STATUS_S, "Station-Enabled,Disconnected-Retrying");
                        m_currentState = STATION_RETRY;
                    }
                }
                break;
        }
    }
private:
    enum State { STOPPED = 0, AP_MODE, STATION_CONNECTING, STATION_CONNECTED, STATION_RETRY, MOVE_TO_SAFE_MODE, CONNECTION_CHECK };
    State m_currentState;
    bool m_verbose;
    int m_lastConnectAttempt, m_lastStatusCheck, m_connectionRetries, m_retryDelay, m_statusDelay, m_connectDelay;
    unsigned long m_lastScanTime;
    bool m_scanStarted;

    void apSetup(void) {
        String ssid, password;
        logger::LoggerConfig.GetConfigString(logger::Config::ConfigParam::CONFIG_AP_SSID_S, ssid);
        logger::LoggerConfig.GetConfigString(logger::Config::ConfigParam::CONFIG_AP_PASSWD_S, password);
        if (ssid.length() == 0) ssid = "wibl-config";
        if (password.length() == 0) password = "wibl-config-password";
        WiFi.softAP(ssid.c_str(), password.c_str());
        WiFi.setSleep(false);
        IPAddress addr = WiFi.softAPIP();
        logger::LoggerConfig.SetConfigString(logger::Config::ConfigParam::CONFIG_WIFIIP_S, addr.toString());
        startMDNSResponder();
    }

    void startMDNSResponder(void) {
        String name;
        logger::LoggerConfig.GetConfigString(logger::Config::CONFIG_MDNS_NAME_S, name);
        MDNS.begin(name);
    }

    bool attemptStationJoin(void) {
        String ssid, password;
        logger::LoggerConfig.GetConfigString(logger::Config::ConfigParam::CONFIG_STATION_SSID_S, ssid);
        logger::LoggerConfig.GetConfigString(logger::Config::ConfigParam::CONFIG_STATION_PASSWD_S, password);
        if (ssid.length() == 0) return false;
        if (m_verbose) {
            WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info){
                Serial.printf("DBG: WiFi Event: %d\n", event);
                if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) Serial.printf("DBG: Disconnect reason: %d\n", info.wifi_sta_disconnected.reason);
                else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) Serial.printf("DBG: Got IP: %s\n", IPAddress(info.got_ip.ip_info.ip.addr).toString().c_str());
            });
        }
        WiFi.mode(WIFI_STA); WiFi.disconnect(true); delay(100); WiFi.setSleep(false);
        wifi_config_t conf; memset(&conf, 0, sizeof(conf));
        memcpy(conf.sta.ssid, ssid.c_str(), ssid.length());
        memcpy(conf.sta.password, password.c_str(), password.length());
        conf.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
        conf.sta.pmf_cfg.capable = true; conf.sta.pmf_cfg.required = true;
        esp_wifi_set_config(WIFI_IF_STA, &conf);
        esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
        wl_status_t status = WiFi.begin();
        m_lastConnectAttempt = millis();
        return status == WL_CONNECTED;
    }

    void completeStationJoin(void) {
        IPAddress addr = WiFi.localIP();
        logger::LoggerConfig.SetConfigString(logger::Config::ConfigParam::CONFIG_WIFIIP_S, addr.toString());
        startMDNSResponder();
    }

    bool isConnected(void) {
        m_lastStatusCheck = millis();
        return WiFi.status() == WL_CONNECTED;
    }

    int maximumReties(void) { String v; logger::LoggerConfig.GetConfigString(logger::Config::ConfigParam::CONFIG_STATION_RETRIES_S, v); return v.toInt(); }
    int retryDelay(void) { String v; logger::LoggerConfig.GetConfigString(logger::Config::ConfigParam::CONFIG_STATION_DELAY_S, v); return v.toInt() * 1000; }
    int connectionDelay(void) { String v; logger::LoggerConfig.GetConfigString(logger::Config::ConfigParam::CONFIG_STATION_TIMEOUT_S, v); return v.toInt() * 1000; }
};

class ESP32WiFiAdapter : public WiFiAdapter {
public:
    ESP32WiFiAdapter(void) : m_storage(mem::MemControllerFactory::Create()), m_server(nullptr), m_messages(new DynamicJsonDocument(1024)), m_statusCode(HTTPReturnCodes::OK), m_verbose(false), m_state(m_verbose) {}
    virtual ~ESP32WiFiAdapter(void) { stop(); delete m_storage; delete m_messages; }
    void RunLoop(void) override { m_state.Verbose(m_verbose); m_state.StepState(); if (m_server) m_server->handleClient(); }
    void SetVerbose(bool v) override { m_verbose = v; m_state.Verbose(v); }
    bool Verbose(void) override { return m_verbose; }
private:
    mem::MemController *m_storage;
    ExtendedWebServer *m_server;
    std::queue<String> m_commands;
    DynamicJsonDocument *m_messages;
    HTTPReturnCodes m_statusCode;
    ConnectionStateMachine m_state;
    bool m_verbose;

    void handleCommand(void) { for (uint32_t i = 0; i < m_server->args(); ++i) if (m_server->argName(i) == "command") m_commands.push(m_server->arg(i)); }
    void heartbeat(void) { m_commands.push("status"); }
    void transferLogs(void) { if (m_server->StreamArchive(m_storage->ControllerPtr(), "/logs") == 0) m_server->send(400, "application/json", "{\"error\":\"no logs\"}"); }
    bool start(void) override {
        m_server = new ExtendedWebServer();
        m_server->on("/heartbeat", HTTPMethod::HTTP_GET, std::bind(&ESP32WiFiAdapter::heartbeat, this));
        m_server->on("/command", HTTPMethod::HTTP_POST, std::bind(&ESP32WiFiAdapter::handleCommand, this));
        m_server->on("/archive", HTTPMethod::HTTP_GET, std::bind(&ESP32WiFiAdapter::transferLogs, this));
        m_server->serveStatic("/logs", m_storage->Controller(), "/logs/");
        m_server->serveStatic("/", LittleFS, "/website/");
        m_state.Start(); m_server->begin(); return true;
    }
    void stop(void) override { delete m_server; m_server = nullptr; }
    String readBuffer(void) override { if (m_commands.empty()) return ""; String r = m_commands.front(); m_commands.pop(); return r; }
    bool sendLogFile(String const& n, uint32_t s, logger::Manager::MD5Hash const& h) override {
        File f = m_storage->Controller().open(n, FILE_READ);
        if (!f) return false;
        m_server->sendHeader("Digest", "md5=" + h.Value());
        m_server->streamFile(f, "application/octet-stream");
        f.close(); return true;
    }
    void accumulateMessage(String const& m) override {
        if ((m_messages->memoryUsage() + m.length()) > 0.95*m_messages->capacity()) {
            DynamicJsonDocument *n = new DynamicJsonDocument(m_messages->capacity() * 2);
            n->set(*m_messages); delete m_messages; m_messages = n;
        }
        (*m_messages)["messages"].add(m);
    }
    void setMessage(DynamicJsonDocument const& m) override { *m_messages = m; }
    void setStatusCode(HTTPReturnCodes c) override { m_statusCode = c; }
    bool transmitMessages(void) override {
        String m; serializeJson(*m_messages, m);
        m_server->send(m_statusCode, "application/json", m);
        m_messages->clear(); m_statusCode = HTTPReturnCodes::OK; return true;
    }
};
#endif

bool WiFiAdapter::Startup(void) { return start(); }
void WiFiAdapter::Shutdown(void) { stop(); }
String WiFiAdapter::ReceivedString(void) { return readBuffer(); }
bool WiFiAdapter::TransferFile(String const& n, uint32_t s, logger::Manager::MD5Hash const& h) { return sendLogFile(n, s, h); }
void WiFiAdapter::AddMessage(String const& m) { accumulateMessage(m); }
void WiFiAdapter::SetMessage(DynamicJsonDocument const& m) { setMessage(m); }
void WiFiAdapter::SetStatusCode(HTTPReturnCodes c) { setStatusCode(c); }
bool WiFiAdapter::TransmitMessages(void) { return transmitMessages(); }

void WiFiAdapter::SetWirelessMode(WirelessMode m) {
    String v = (m == WirelessMode::ADAPTER_STATION) ? "Station" : "AP";
    logger::LoggerConfig.SetConfigString(logger::Config::ConfigParam::CONFIG_WIFIMODE_S, v);
}

WiFiAdapter::WirelessMode WiFiAdapter::GetWirelessMode(void) {
    String v; logger::LoggerConfig.GetConfigString(logger::Config::ConfigParam::CONFIG_WIFIMODE_S, v);
    return (v == "Station") ? WirelessMode::ADAPTER_STATION : WirelessMode::ADAPTER_SOFTAP;
}

WiFiAdapter *WiFiAdapterFactory::Create(void) {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    return new ESP32WiFiAdapter();
#else
    return nullptr;
#endif
}
