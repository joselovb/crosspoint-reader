#pragma once

#include <Memory.h>
#include <SecureHttpClient.h>

#include "activities/Activity.h"

// X4 Lab "Terminal": polls the bridge (x4-lab/bridge) for a full-screen 1-bit
// frame and blits it raw into the framebuffer. Frame format "X4" v1: 8-byte
// header (magic, version, flags, frame id LE) + 48000 panel-native bytes.
// Config lives on the SD card at /x4lab/config.json.
class TerminalActivity final : public Activity {
 public:
  explicit TerminalActivity(GfxRenderer& renderer, MappedInputManager& mappedInput)
      : Activity("X4Terminal", renderer, mappedInput) {}

  void onEnter() override;
  void onExit() override;
  void loop() override;
  void render(RenderLock&&) override;
  bool preventAutoSleep() override { return state == SHOWING; }

 private:
  static constexpr size_t HEADER_SIZE = 8;
  static constexpr size_t PIXELS_SIZE = 48000;
  static constexpr size_t FRAME_SIZE = HEADER_SIZE + PIXELS_SIZE;
  static constexpr uint8_t FLAG_FULL_REFRESH = 2;

  enum State { CONNECTING, SHOWING, NO_CONFIG, FAILED };
  State state = CONNECTING;

  struct Config {
    char url[128] = "";
    char token[96] = "";
    char orient[12] = "portrait";
    bool tlsVerify = false;
    uint32_t pollMs = 2000;
    uint16_t fullRefreshEvery = 20;
  } config;

  // Downloaded frame; written by loop() and read by render(), both under RenderLock.
  std::unique_ptr<uint8_t[]> frame;
  size_t received = 0;
  int64_t frameId = -1;
  bool frameDirty = false;
  uint16_t framesSinceFull = 0;
  int lastStatus = 0;
  unsigned long lastPollMs = 0;
  bool shouldTearDownWifiOnExit = false;
  freeink::SecureHttpClient http;

  bool loadConfig();
  void poll();
  void launchWifiSelection();
};
