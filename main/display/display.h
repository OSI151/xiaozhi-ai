#ifndef DISPLAY_H
#define DISPLAY_H

#include "emoji_collection.h"

#ifndef CONFIG_USE_EMOTE_MESSAGE_STYLE
#define HAVE_LVGL 1
#include <lvgl.h>
#endif

#include <esp_timer.h>
#include <esp_log.h>
#include <esp_pm.h>

#include <string>
#include <chrono>

class Theme {
public:
    Theme(const std::string& name) : name_(name) {}
    virtual ~Theme() = default;

    inline std::string name() const { return name_; }
private:
    std::string name_;
};

class Display {
public:
    Display();
    virtual ~Display();

    virtual void SetStatus(const char* status);
    virtual void ShowNotification(const char* notification, int duration_ms = 3000);
    virtual void ShowNotification(const std::string &notification, int duration_ms = 3000);
    virtual void SetEmotion(const char* emotion);
    virtual void SetChatMessage(const char* role, const char* content);
    virtual void ClearChatMessages();
    virtual void SetTheme(Theme* theme);
    virtual Theme* GetTheme() { return current_theme_; }
    virtual void UpdateStatusBar(bool update_all = false);
    virtual void SetPowerSaveMode(bool on);
    virtual void SetupUI() { 
        setup_ui_called_ = true;
    }

    // === 第 2 步：眼睛动画（RoboEyes）接口 ===
    // OLED 上开启后，RoboEyes 会用自己开的 FreeRTOS 任务直接刷 panel，
    // 不经过 LVGL。基类留空实现，非 OLED 的显示对象调用无副作用。
    virtual void SetAnimatedEmotionMode(bool enable);
    virtual bool IsAnimatedEmotionMode() const { return animated_emotion_mode_; }
    virtual void SetEmotionDirection(int direction);  // 0-8: 中间/上/下/左/右/四个斜向
    virtual void UpdateAnimatedEmotion();

    // === 第 6 步：整屏大字时钟（睡眠态）===
    // OLED 上占满整屏，与状态栏/表情/字幕互斥。基类留空实现。
    virtual void ShowClock();
    virtual void HideClock();
    virtual bool IsClockVisible() const { return false; }

    // === 7b：音乐界面 ===
    // 固件里没有音乐服务，歌名和歌词是从服务端推下来的 sentence_start 文本里
    // 认出来的（"% play_music..." → "《歌名》" → 歌词）。这是依赖服务端格式的
    // hack，认不出来就当普通字幕走，不会崩。基类留空实现。
    // on=true 时 title/lyric 都可能为空（刚进界面还没拿到歌名）。
    virtual void SetMusicMode(bool on, const char* title, const char* lyric);
    virtual bool IsMusicMode() const { return music_mode_; }

    inline int width() const { return width_; }
    inline int height() const { return height_; }
    inline bool IsSetupUICalled() const { return setup_ui_called_; }

protected:
    int width_ = 0;
    int height_ = 0;
    bool setup_ui_called_ = false;  // Track if SetupUI() has been called
    bool animated_emotion_mode_ = false;
    bool music_mode_ = false;

    Theme* current_theme_ = nullptr;

    friend class DisplayLockGuard;
    virtual bool Lock(int timeout_ms = 0) = 0;
    virtual void Unlock() = 0;
};


class DisplayLockGuard {
public:
    DisplayLockGuard(Display *display) : display_(display) {
        // 原来这里是 Lock(30000) 并且失败只打日志。两个问题：
        //   1) 30 秒太长，一旦拿不到就把调用者（可能是 esp_timer 任务）拖死，WDT 直接触发；
        //   2) 失败后析构仍然 Unlock()，而 lvgl_port_unlock 是 xSemaphoreGiveRecursive，
        //      等于把别人持有的递归锁计数减掉，锁状态从此错乱，之后所有人都拿不到。
        // 现在：超时降到 1 秒，并且只有真正拿到锁才在析构里释放。
        locked_ = display_->Lock(1000);
        if (!locked_) {
            ESP_LOGW("Display", "Failed to lock display within 1000 ms");
        }
    }
    ~DisplayLockGuard() {
        if (locked_) {
            display_->Unlock();
        }
    }
    bool locked() const { return locked_; }

private:
    Display *display_ = nullptr;
    bool locked_ = false;
};

class NoDisplay : public Display {
private:
    virtual bool Lock(int timeout_ms = 0) override {
        return true;
    }
    virtual void Unlock() override {}
};

#endif
