#include "TerminalActivity.h"

#include <ArduinoJson.h>
#include <GfxRenderer.h>
#include <HalStorage.h>
#include <I18n.h>
#include <Logging.h>
#include <WiFi.h>

#include <cstdio>
#include <cstring>

#include "IsrgRootX1.h"
#include "MappedInputManager.h"
#include "SilentRestart.h"
#include "activities/network/WifiSelectionActivity.h"
#include "components/UITheme.h"
#include "fontIds.h"

namespace {
constexpr char CONFIG_PATH[] = "/x4lab/config.json";
constexpr size_t CONFIG_MAX_BYTES = 2048;
constexpr uint32_t HTTP_TIMEOUT_MS = 10000;
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
  config.fullRefreshEvery = doc["full_refresh_every"] | 20;
  const size_t urlLen = strlen(config.url);
  if (urlLen > 0 && config.url[urlLen - 1] == '/') config.url[urlLen - 1] = '\0';
  if (config.url[0] == '\0' || config.token[0] == '\0') {
    LOG_ERR("X4T", "%s needs url and token", CONFIG_PATH);
    return false;
  }
  LOG_INF("X4T", "Bridge %s, tls=%s, poll=%ums", config.url, config.tlsVerify ? "verify" : "insecure",
          static_cast<unsigned>(config.pollMs));
  return true;
}

void TerminalActivity::onEnter() {
  Activity::onEnter();
  state = CONNECTING;
  frameId = -1;

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

void TerminalActivity::poll() {
  lastPollMs = millis();
  if (WiFi.status() != WL_CONNECTED) return;

  // ~300 bytes: over the 256-byte guideline, but this runs on the main loop task
  // (not a small worker stack) and avoids a heap string per poll.
  char url[sizeof(config.url) + sizeof(config.token) + 80];
  snprintf(url, sizeof(url), "%s/api/v1/screen?app=terminal&orient=%s&since=%lld&t=%s", config.url, config.orient,
           static_cast<long long>(frameId), config.token);

  if (config.tlsVerify) {
    http.setCACert(ISRG_ROOT_X1_PEM);
  } else {
    http.setInsecure();
  }
  http.setTimeout(HTTP_TIMEOUT_MS);

  int status = -1;
  bool valid = false;
  {
    // render() reads `frame`; hold the lock while the body streams into it.
    RenderLock lock(*this);
    received = 0;
    if (http.begin(url)) {
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

  if (status == 204) return;  // unchanged
  if (valid) {
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

void TerminalActivity::loop() {
  if (mappedInput.wasPressed(MappedInputManager::Button::Back)) {
    onGoHome();
    return;
  }
  if (state == NO_CONFIG || !frame) return;
  if (lastPollMs == 0 || millis() - lastPollMs >= config.pollMs) poll();
}

void TerminalActivity::render(RenderLock&&) {
  if (state == SHOWING) {
    if (!frameDirty) return;
    if (renderer.getBufferSize() != PIXELS_SIZE) {
      LOG_ERR("X4T", "Unexpected framebuffer size %u", static_cast<unsigned>(renderer.getBufferSize()));
      return;
    }
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
