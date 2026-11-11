#include <esp_log.h>
#include <esp_err.h>
#include <string>
#include <cstdlib>
#include <cstring>
#include <font_awesome.h>

#include "display.h"
#include "board.h"
#include "application.h"
#include "audio_codec.h"
#include "settings.h"
#include "assets/lang_config.h"

#define TAG "Display"

Display::Display() {
}

Display::~Display() {
}

void Display::SetStatus(const char* status) {
    ESP_LOGW(TAG, "SetStatus: %s", status);
}

void Display::ShowNotification(const std::string &notification, int duration_ms) {
    ShowNotification(notification.c_str(), duration_ms);
}

void Display::ShowNotification(const char* notification, int duration_ms) {
    ESP_LOGW(TAG, "ShowNotification: %s", notification);
}

void Display::UpdateStatusBar(bool update_all) {
}


void Display::SetEmotion(const char* emotion) {
    ESP_LOGW(TAG, "SetEmotion: %s", emotion);
}

void Display::SetChatMessage(const char* role, const char* content) {
    ESP_LOGW(TAG, "Role:%s", role);
    ESP_LOGW(TAG, "     %s", content);
}

void Display::ClearChatMessages() {
    // Default empty implementation, override in subclasses if needed
}

void Display::SetTheme(Theme* theme) {
    current_theme_ = theme;
    Settings settings("display", true);
    settings.SetString("theme", theme->name());
}

void Display::SetPowerSaveMode(bool on) {
    ESP_LOGW(TAG, "SetPowerSaveMode: %d", on);
}

// === 第 2 步：眼睛动画接口的基类空实现 ===
// 真实现只在 OledDisplay 里（RoboEyes 直接刷 panel），其它显示对象调用这些
// 方法是安全的空操作，这样 application.cc 可以无脑对所有 Display 调用。

void Display::SetAnimatedEmotionMode(bool enable) {
    animated_emotion_mode_ = enable;
}

void Display::SetEmotionDirection(int direction) {
    (void)direction;
}

void Display::UpdateAnimatedEmotion() {
}

// === 第 6 步：整屏大字时钟 ===
// 只有 OLED 有实现（真在屏上画 label）。其它显示对象是空操作，
// 这样 application.cc 可以对任意 Display 调 ShowClock/HideClock。

void Display::ShowClock() {
}

void Display::HideClock() {
}

// === 7b：音乐界面 ===
// 同样只有 OLED 有实现。

void Display::SetMusicMode(bool on, const char* title, const char* lyric) {
    music_mode_ = on;
    (void)title;
    (void)lyric;
}
