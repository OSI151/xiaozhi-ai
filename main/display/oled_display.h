#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "lvgl_display.h"
#include "roboeyes_adapter.h"

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>

#include <memory>


class OledDisplay : public LvglDisplay {
private:
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;

    lv_obj_t* top_bar_ = nullptr;
    lv_obj_t* status_bar_ = nullptr;
    lv_obj_t* content_ = nullptr;
    lv_obj_t* content_left_ = nullptr;
    lv_obj_t* content_right_ = nullptr;
    lv_obj_t* container_ = nullptr;
    lv_obj_t* side_bar_ = nullptr;
    lv_obj_t *emotion_label_ = nullptr;
    lv_obj_t* chat_message_label_ = nullptr;
    // 分屏专用字幕：眼睛占 0~47 行，这个贴在 48~63 行，吃满 128 宽。
    // 只在眼睛模式下创建和使用，官方界面仍走 chat_message_label_。
    lv_obj_t* subtitle_label_ = nullptr;

    // 第 6 步：整屏大字时钟。只在 kDisplayModeSleep 态显示，
    // 和 container_（状态栏 + 表情 + 字幕）互斥。
    // 拆成 时 / 冒号 / 分 三个 label 才能让冒号单独闪——单个 label 里
    // 没法只改一个字符的透明度。
    lv_obj_t* clock_row_   = nullptr;
    lv_obj_t* clock_h_     = nullptr;
    lv_obj_t* clock_colon_ = nullptr;
    lv_obj_t* clock_m_     = nullptr;
    // 时钟当前是否已显示。ShowClock()/HideClock() 用它做幂等判断，
    // 避免每秒都改一次 flag 触发整屏重绘。
    bool clock_visible_    = false;
    lv_obj_t* date_label_  = nullptr;

    // === 7b：音乐界面 ===
    // 128×64 分四段：第 0 页 "♪ 正在播放"，第 1-2 页歌名，第 3-6 页当前歌词，
    // 第 7 页留空。歌词只显示当前一句，超长直接截断——16px 高跑马灯读着累。
    lv_obj_t* music_title_  = nullptr;
    lv_obj_t* music_lyric_  = nullptr;
    bool music_visible_ = false;

    // 第 2 步：RoboEyes 直接刷 panel 的适配器
    std::unique_ptr<RoboEyesAdapter> roboeyes_adapter_;

    virtual bool Lock(int timeout_ms = 0) override;
    virtual void Unlock() override;

    void SetupUI_128x64();
    void SetupUI_128x32();

public:
    OledDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel, int width, int height, bool mirror_x, bool mirror_y);
    ~OledDisplay();

    virtual void SetupUI() override;
    virtual void SetChatMessage(const char* role, const char* content) override;
    virtual void ClearChatMessages() override;
    virtual void SetEmotion(const char* emotion) override;
    virtual void SetTheme(Theme* theme) override;

    // 第 2 步：眼睛动画
    virtual void SetAnimatedEmotionMode(bool enable) override;
    virtual void SetEmotionDirection(int direction) override;
    virtual void UpdateAnimatedEmotion() override;

    // 第 6 步：整屏大字时钟（睡眠态）
    virtual void ShowClock() override;
    virtual void HideClock() override;
    virtual bool IsClockVisible() const override { return clock_visible_; }

    // 7b：音乐界面
    virtual void SetMusicMode(bool on, const char* title, const char* lyric) override;
    virtual bool IsMusicMode() const override { return music_visible_; }

private:
    void UpdateClockText();
};

#endif // OLED_DISPLAY_H
