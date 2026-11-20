#include "TerminalActivity.h"

#include <ArduinoJson.h>
#include <GfxRenderer.h>
#include <HalStorage.h>
#include <I18n.h>
#include <Logging.h>
#include <WiFi.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>

#include "IsrgRootX1.h"
#include "MappedInputManager.h"
#include "SilentRestart.h"
#include "activities/network/WifiSelectionActivity.h"
#include "activities/util/KeyboardEntryActivity.h"
#include "components/HeaderBackTapTarget.h"
#include "components/UITheme.h"
#include "fontIds.h"

namespace {
constexpr char CONFIG_PATH[] = "/x4lab/config.json";
constexpr size_t CONFIG_MAX_BYTES = 2048;
constexpr uint32_t HTTP_TIMEOUT_MS = 10000;
constexpr unsigned long LONG_PRESS_MS = 600;
// After sending keys, poll this soon: the bridge publishes once the screen holds
// still for 0.5 s.
constexpr unsigned long KICK_MS = 700;
constexpr size_t MAX_TEXT = 500;  // bridge MAX_TEXT

// Touch bar the bridge draws at the bottom of the frame: equal-width cells,
// BAR_ROW_H tall, tmux key names. Keep in sync with BAR in x4-lab bridge/app.py.
constexpr int BAR_ROWS = 2;
constexpr int BAR_COLS = 6;
constexpr int BAR_ROW_H = 48;
constexpr char KBD[] = "KBD";  // opens the on-screen keyboard instead
constexpr const char* BAR_KEYS[BAR_ROWS][BAR_COLS] = {
    {"1", "2", "3", "Up", "Escape", KBD},
    {"BTab", "PageUp", "PageDown", "Down", "Tab", "Enter"},
};

void appendUrlEncoded(std::string& out, const std::string& in) {
  static constexpr char HEX_DIGITS[] = "0123456789ABCDEF";
  for (const unsigned char c : in) {
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      out += static_cast<char>(c);
    } else {
      out += '%';
      out += HEX_DIGITS[c >> 4];
      out += HEX_DIGITS[c & 0xF];
    }
  }
}
}  // namespace

bool TerminalActivity::loadConfig() {
  auto buf = makeUniqueNoThrow<char[]>(CONFIG_MAX_BYTES);
  if (!buf) {
    LOG_ERR("X4T", "OOM: config buffer");
    return false;
  }
  const size_t len = Storage.readFileToBuffer(CONFIG_PATH, buf.get(), CONFIG_MAX_BYTES);
  if (len == 0) {
    LOG_ERR("X4T", "Missing or empty %s", CONFIG_PATH);
    return false;
  }
  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, buf.get(), len);
  if (err) {
    LOG_ERR("X4T", "Bad %s: %s", CONFIG_PATH, err.c_str());
    return false;
  }
  snprintf(config.url, sizeof(config.url), "%s", doc["url"] | "");
  snprintf(config.token, sizeof(config.token), "%s", doc["token"] | "");
  snprintf(config.orient, sizeof(config.orient), "%s", doc["orient"] | "portrait");
  config.tlsVerify = strcmp(doc["tls"] | "insecure", "verify") == 0;
  config.pollMs = doc["poll_ms"] | 2000;
  config.idlePollMs = doc["idle_poll_ms"] | 20000;
  config.idleAfterMs = (doc["idle_after_s"] | 180) * 1000UL;
  config.fullRefreshEvery = doc["full_refresh_every"] | 20;
  const size_t urlLen = strlen(config.url);
  if (urlLen > 0 && config.url[urlLen - 1] == '/') config.url[urlLen - 1] = '\0';
  if (config.url[0] == '\0' || config.token[0] == '\0') {
    LOG_ERR("X4T", "%s needs url and token", CONFIG_PATH);
    return false;
  }
  LOG_INF("X4T", "Bridge %s, tls=%s, poll=%ums, idle poll=%ums after %us", config.url,
          config.tlsVerify ? "verify" : "insecure", static_cast<unsigned>(config.pollMs),
          static_cast<unsigned>(config.idlePollMs), static_cast<unsigned>(config.idleAfterMs / 1000));
  return true;
}

void TerminalActivity::onEnter() {
  Activity::onEnter();
  state = CONNECTING;
  frameId = -1;
  lastActiveMs = millis();

  if (!loadConfig()) {
    state = NO_CONFIG;
    requestUpdate();
    return;
  }
  // Allocated once for the activity's lifetime; PSRAM-backed malloc on the X4 Pro.
  frame = makeUniqueNoThrow<uint8_t[]>(FRAME_SIZE);
  if (!frame) {
    LOG_ERR("X4T", "OOM: frame buffer (%u bytes)", static_cast<unsigned>(FRAME_SIZE));
    lastStatus = -1;
    state = FAILED;
    requestUpdate();
    return;
  }
  framesSinceFull = config.fullRefreshEvery;  // first frame gets a clean refresh

  requestUpdate();
  if (WiFi.status() != WL_CONNECTED) {
    shouldTearDownWifiOnExit = true;
    launchWifiSelection();
  }
}

void TerminalActivity::onExit() {
  Activity::onExit();
  http.end();
  frame.reset();

  if (shouldTearDownWifiOnExit && WiFi.getMode() != WIFI_MODE_NULL) {
    WiFi.disconnect(false);
    delay(30);
    silentRestart();
  }
}

void TerminalActivity::launchWifiSelection() {
  startActivityForResult(std::make_unique<WifiSelectionActivity>(renderer, mappedInput),
                         [this](const ActivityResult& result) {
                           if (result.isCancelled) {
                             onGoHome();
                             return;
                           }
                           lastPollMs = 0;  // poll right away
                         });
}

bool TerminalActivity::beginRequest(const char* url) {
  if (config.tlsVerify) {
    http.setCACert(ISRG_ROOT_X1_PEM);
  } else {
    http.setInsecure();
  }
  http.setTimeout(HTTP_TIMEOUT_MS);
  return http.begin(url);
}

void TerminalActivity::poll() {
  lastPollMs = millis();
  if (WiFi.status() != WL_CONNECTED) return;

  // ~300 bytes: over the 256-byte guideline, but this runs on the main loop task
  // (not a small worker stack) and avoids a heap string per poll.
  char url[sizeof(config.url) + sizeof(config.token) + 80];
  snprintf(url, sizeof(url), "%s/api/v1/screen?app=terminal&orient=%s&since=%lld&t=%s", config.url, config.orient,
           static_cast<long long>(frameId), config.token);

  int status = -1;
  bool valid = false;
  {
    // render() reads `frame`; hold the lock while the body streams into it.
    RenderLock lock(*this);
    received = 0;
    if (beginRequest(url)) {
      status = http.GET([this](const uint8_t* data, size_t len) {
        if (received + len > FRAME_SIZE) return false;
        memcpy(frame.get() + received, data, len);
        received += len;
        return true;
      });
    }
    if (status == 200) {
      valid = received == FRAME_SIZE && frame[0] == 'X' && frame[1] == '4' && frame[2] == 1;
      if (valid) {
        uint32_t id;
        memcpy(&id, frame.get() + 4, sizeof(id));
        frameId = id;
        frameDirty = true;
      }
    }
  }

  // Unchanged: 204, or 200 with only the header (what the bridge sends, since the
  // web emulator's service worker cannot pass a 204 through).
  if (status == 204 || (status == 200 && received == HEADER_SIZE && frame[0] == 'X' && frame[1] == '4')) return;
  if (valid) {
    lastActiveMs = millis();
    if (state != SHOWING) framesSinceFull = config.fullRefreshEvery;  // clear the text screen fully
    state = SHOWING;
    requestUpdate();
    return;
  }
  LOG_ERR("X4T", "Poll failed: status=%d received=%u", status, static_cast<unsigned>(received));
  if (state != FAILED || status != lastStatus) {
    lastStatus = status;
    state = FAILED;
    requestUpdate();
  }
}

// query: "k=Up&k=Enter" or "text=...&enter=1", already URL-encoded.
void TerminalActivity::sendKeys(const std::string& query) {
  if (WiFi.status() != WL_CONNECTED) return;
  std::string url = config.url;
  url += "/api/v1/keys?";
  url += query;
  url += "&t=";
  url += config.token;
  int status = -1;
  if (beginRequest(url.c_str())) status = http.GET([](const uint8_t*, size_t) { return true; });
  if (status != 200) {
    LOG_ERR("X4T", "Keys failed: %s status=%d", query.c_str(), status);
  } else {
    LOG_DBG("X4T", "Keys %s", query.c_str());
  }
  // Poll soon instead of waiting a full cycle (like kick() in the web viewer).
  lastPollMs = millis() - config.pollMs + KICK_MS;
}

void TerminalActivity::openKeyboard() {
  startActivityForResult(
      std::make_unique<KeyboardEntryActivity>(renderer, mappedInput, tr(STR_X4LAB_SEND_TEXT), "", MAX_TEXT),
      [this](const ActivityResult& result) {
        // The keyboard drew over the whole screen: repaint the last frame cleanly.
        frameDirty = state == SHOWING;
        framesSinceFull = config.fullRefreshEvery;
        if (result.isCancelled) return;
        const auto& text = std::get<KeyboardResult>(result.data).text;
        if (text.empty()) return;
        std::string query = "text=";
        appendUrlEncoded(query, text);
        query += "&enter=1";
        sendKeys(query);
      });
}

void TerminalActivity::handleInput() {
  using Button = MappedInputManager::Button;
  // Side buttons: Up/Down, long press = PageUp/PageDown (a long press consumes the release).
  if (mappedInput.wasLongPressed(Button::Up, LONG_PRESS_MS)) return sendKeys("k=PageUp");
  if (mappedInput.wasLongPressed(Button::Down, LONG_PRESS_MS)) return sendKeys("k=PageDown");
  if (mappedInput.wasReleased(Button::Up)) return sendKeys("k=Up");
  if (mappedInput.wasReleased(Button::Down)) return sendKeys("k=Down");

  int x = 0;
  int y = 0;
  if (state != SHOWING || !mappedInput.wasScreenTapped(x, y)) return;
  const int top = renderer.getScreenHeight() - BAR_ROWS * BAR_ROW_H;
  if (y < top) return;
  const int row = std::min((y - top) / BAR_ROW_H, BAR_ROWS - 1);
  const int col = std::min(x / (renderer.getScreenWidth() / BAR_COLS), BAR_COLS - 1);
  const char* key = BAR_KEYS[row][col];
  if (strcmp(key, KBD) == 0) return openKeyboard();
  sendKeys(std::string("k=") + key);
}

// Slow down while nothing happens, but never while Claude waits for an answer.
uint32_t TerminalActivity::pollInterval() const {
  const bool waiting = frameId >= 0 && (frame[3] & FLAG_WAITING);
  return !waiting && millis() - lastActiveMs >= config.idleAfterMs ? config.idlePollMs : config.pollMs;
}

void TerminalActivity::loop() {
  if (mappedInput.wasPressed(MappedInputManager::Button::Back)) {
    onGoHome();
    return;
  }
  if (state == NO_CONFIG || !frame) return;
  int x = 0;
  int y = 0;
  if (mappedInput.wasPressed(MappedInputManager::Button::Up) || mappedInput.wasPressed(MappedInputManager::Button::Down) ||
      mappedInput.wasScreenTouchDown(x, y)) {
    if (pollInterval() != config.pollMs) lastPollMs = 0;  // waking from idle: poll now
    lastActiveMs = millis();
  }
  handleInput();
  const uint32_t interval = pollInterval();
  if (interval != lastInterval) {
    LOG_DBG("X4T", "Polling every %u ms", static_cast<unsigned>(interval));
    lastInterval = interval;
  }
  if (lastPollMs == 0 || millis() - lastPollMs >= interval) poll();
}

void TerminalActivity::render(RenderLock&&) {
  if (state == SHOWING) {
    if (!frameDirty) return;
    if (renderer.getBufferSize() != PIXELS_SIZE) {
      LOG_ERR("X4T", "Unexpected framebuffer size %u", static_cast<unsigned>(renderer.getBufferSize()));
      return;
    }
    // The connecting screen's header registered a back tap rect; the frame has no header.
    HeaderBackTapTarget::clear();
    memcpy(renderer.getFrameBuffer(), frame.get() + HEADER_SIZE, PIXELS_SIZE);
    frameDirty = false;
    const bool full = ++framesSinceFull >= config.fullRefreshEvery || (frame[3] & FLAG_FULL_REFRESH);
    if (full) framesSinceFull = 0;
    renderer.displayBuffer(full ? HalDisplay::HALF_REFRESH : HalDisplay::FAST_REFRESH);
    return;
  }

  const auto& metrics = UITheme::getInstance().getMetrics();
  const auto pageWidth = renderer.getScreenWidth();
  const int midY = renderer.getScreenHeight() / 2;
  renderer.clearScreen();
  GUI.drawHeader(renderer, Rect{0, metrics.topPadding, pageWidth, metrics.headerHeight}, tr(STR_X4LAB_TERMINAL));
  switch (state) {
    case CONNECTING:
      renderer.drawCenteredText(UI_12_FONT_ID, midY, tr(STR_X4LAB_CONNECTING));
      break;
    case NO_CONFIG:
      renderer.drawCenteredText(UI_12_FONT_ID, midY - 20, tr(STR_X4LAB_NO_CONFIG), true, EpdFontFamily::BOLD);
      renderer.drawCenteredText(UI_10_FONT_ID, midY + 10, CONFIG_PATH);
      break;
    case FAILED: {
      char line[32];
      snprintf(line, sizeof(line), "HTTP %d", lastStatus);
      renderer.drawCenteredText(UI_12_FONT_ID, midY - 20, tr(STR_X4LAB_FAILED), true, EpdFontFamily::BOLD);
      renderer.drawCenteredText(UI_10_FONT_ID, midY + 10, line);
      break;
    }
    case SHOWING:
      break;
  }
  const auto labels = mappedInput.mapLabels(tr(STR_BACK), "", "", "");
  GUI.drawButtonHints(renderer, labels.btn1, labels.btn2, labels.btn3, labels.btn4);
  renderer.displayBuffer();
}
