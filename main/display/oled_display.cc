#include "oled_display.h"
#include "assets/lang_config.h"
#include "lvgl_theme.h"
#include "lvgl_font.h"

#include <string>
#include <algorithm>

#include <esp_log.h>
#include <esp_err.h>
#include <esp_lvgl_port.h>
#include <font_awesome.h>

#define TAG "OledDisplay"

LV_FONT_DECLARE(BUILTIN_TEXT_FONT);
LV_FONT_DECLARE(BUILTIN_ICON_FONT);
LV_FONT_DECLARE(font_awesome_30_1);

OledDisplay::OledDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
    int width, int height, bool mirror_x, bool mirror_y)
    : panel_io_(panel_io), panel_(panel) {
    width_ = width;
    height_ = height;

    auto text_font = std::make_shared<LvglBuiltInFont>(&BUILTIN_TEXT_FONT);
    auto icon_font = std::make_shared<LvglBuiltInFont>(&BUILTIN_ICON_FONT);
    auto large_icon_font = std::make_shared<LvglBuiltInFont>(&font_awesome_30_1);
    
    auto dark_theme = new LvglTheme("dark");
    dark_theme->set_text_font(text_font);
    dark_theme->set_icon_font(icon_font);
    dark_theme->set_large_icon_font(large_icon_font);

    auto& theme_manager = LvglThemeManager::GetInstance();
    theme_manager.RegisterTheme("dark", dark_theme);
    current_theme_ = dark_theme;

    ESP_LOGI(TAG, "Initialize LVGL");
    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    port_cfg.task_priority = 1;
    port_cfg.task_stack = 6144;
#if CONFIG_SOC_CPU_CORES_NUM > 1
    port_cfg.task_affinity = 1;
#endif
    lvgl_port_init(&port_cfg);

    ESP_LOGI(TAG, "Adding OLED display");
    const lvgl_port_display_cfg_t display_cfg = {
        .io_handle = panel_io_,
        .panel_handle = panel_,
        .control_handle = nullptr,
        .buffer_size = static_cast<uint32_t>(width_ * height_),
        .double_buffer = false,
        .trans_size = 0,
        .hres = static_cast<uint32_t>(width_),
        .vres = static_cast<uint32_t>(height_),
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = mirror_x,
            .mirror_y = mirror_y,
        },
        .flags = {
            .buff_dma = 1,
            .buff_spiram = 0,
            .sw_rotate = 0,
            .full_refresh = 0,
            .direct_mode = 0,
        },
    };

    display_ = lvgl_port_add_disp(&display_cfg);
    if (display_ == nullptr) {
        ESP_LOGE(TAG, "Failed to add display");
        return;
    }

    // Note: SetupUI() should be called by Application::Initialize(), not in constructor
    // to ensure lvgl objects are created after the display is fully initialized.
}

void OledDisplay::SetupUI() {
    // Prevent duplicate calls - if already called, return early
    if (setup_ui_called_) {
        ESP_LOGW(TAG, "SetupUI() called multiple times, skipping duplicate call");
        return;
    }
    
    Display::SetupUI();  // Mark SetupUI as called
    if (height_ == 64) {
        SetupUI_128x64();
    } else {
        SetupUI_128x32();
    }
}

OledDisplay::~OledDisplay() {
    if (content_ != nullptr) {
        lv_obj_del(content_);
    }

    bool is_128x64_layout = (top_bar_ != nullptr);
    if (status_bar_ != nullptr && is_128x64_layout) {
        status_label_ = nullptr;
        notification_label_ = nullptr;
        lv_obj_del(status_bar_);
    }
    if (top_bar_ != nullptr) {
        network_label_ = nullptr;
        mute_label_ = nullptr;
        battery_label_ = nullptr;
        lv_obj_del(top_bar_);
    }
    if (side_bar_ != nullptr) {
        if (!is_128x64_layout) {
            status_label_ = nullptr;
            notification_label_ = nullptr;
            network_label_ = nullptr;
            mute_label_ = nullptr;
            battery_label_ = nullptr;
        }
        lv_obj_del(side_bar_);
    }
    if (container_ != nullptr) {
        lv_obj_del(container_);
    }

    if (panel_ != nullptr) {
        esp_lcd_panel_del(panel_);
    }
    if (panel_io_ != nullptr) {
        esp_lcd_panel_io_del(panel_io_);
    }
    lvgl_port_deinit();
}

bool OledDisplay::Lock(int timeout_ms) {
    return lvgl_port_lock(timeout_ms);
}

void OledDisplay::Unlock() {
    lvgl_port_unlock();
}


// 官方基类是空实现，OLED 一直没 override，所以对话结束的字幕残留在屏上。
void OledDisplay::ClearChatMessages() {
    DisplayLockGuard lock(this);
    if (chat_message_label_ != nullptr) {
        lv_label_set_text(chat_message_label_, "");
    }
}

void OledDisplay::SetChatMessage(const char* role, const char* content) {
    DisplayLockGuard lock(this);
    if (chat_message_label_ == nullptr) {
        return;
    }

    // Replace all newlines with spaces
    std::string content_str = content;
    std::replace(content_str.begin(), content_str.end(), '\n', ' ');

    // 眼睛模式下官方容器是藏着的，字要写到下半屏那条独立字幕上。

    if (content_right_ == nullptr) {
        lv_label_set_text(chat_message_label_, content_str.c_str());
    } else {
        if (content == nullptr || content[0] == '\0') {
            lv_obj_add_flag(content_right_, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_label_set_text(chat_message_label_, content_str.c_str());
            lv_obj_remove_flag(content_right_, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

void OledDisplay::SetupUI_128x64() {
    DisplayLockGuard lock(this);

    auto lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    auto text_font = lvgl_theme->text_font()->font();
    auto icon_font = lvgl_theme->icon_font()->font();
    auto large_icon_font = lvgl_theme->large_icon_font()->font();

    auto screen = lv_screen_active();
    lv_obj_set_style_text_font(screen, text_font, 0);
    lv_obj_set_style_text_color(screen, lv_color_black(), 0);

    /* Container */
    container_ = lv_obj_create(screen);
    lv_obj_set_size(container_, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_flex_flow(container_, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(container_, 0, 0);
    lv_obj_set_style_border_width(container_, 0, 0);
    lv_obj_set_style_pad_row(container_, 0, 0);

    /* Layer 1: Top bar - for status icons */
    top_bar_ = lv_obj_create(container_);
    lv_obj_set_size(top_bar_, LV_HOR_RES, 16);
    lv_obj_set_style_radius(top_bar_, 0, 0);
    lv_obj_set_style_bg_opa(top_bar_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(top_bar_, 0, 0);
    lv_obj_set_style_pad_all(top_bar_, 0, 0);
    lv_obj_set_flex_flow(top_bar_, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top_bar_, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scrollbar_mode(top_bar_, LV_SCROLLBAR_MODE_OFF);

    network_label_ = lv_label_create(top_bar_);
    lv_label_set_text(network_label_, "");
    lv_obj_set_style_text_font(network_label_, icon_font, 0);

    lv_obj_t* right_icons = lv_obj_create(top_bar_);
    lv_obj_set_size(right_icons, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(right_icons, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(right_icons, 0, 0);
    lv_obj_set_style_pad_all(right_icons, 0, 0);
    lv_obj_set_flex_flow(right_icons, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(right_icons, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    mute_label_ = lv_label_create(right_icons);
    lv_label_set_text(mute_label_, "");
    lv_obj_set_style_text_font(mute_label_, icon_font, 0);

    battery_label_ = lv_label_create(right_icons);
    lv_label_set_text(battery_label_, "");
    lv_obj_set_style_text_font(battery_label_, icon_font, 0);

    /* Layer 2: Status bar - for center text labels */
    status_bar_ = lv_obj_create(screen);
    lv_obj_set_size(status_bar_, LV_HOR_RES, 16);
    lv_obj_set_style_radius(status_bar_, 0, 0);
    lv_obj_set_style_bg_opa(status_bar_, LV_OPA_TRANSP, 0);  // Transparent background
    lv_obj_set_style_border_width(status_bar_, 0, 0);
    lv_obj_set_style_pad_all(status_bar_, 0, 0);
    lv_obj_set_scrollbar_mode(status_bar_, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_layout(status_bar_, LV_LAYOUT_NONE, 0);  // Use absolute positioning
    lv_obj_align(status_bar_, LV_ALIGN_TOP_MID, 0, 0);  // Overlap with top_bar_

    notification_label_ = lv_label_create(status_bar_);
    lv_obj_set_width(notification_label_, LV_HOR_RES);
    lv_obj_set_style_text_align(notification_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(notification_label_, "");
    lv_obj_align(notification_label_, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(notification_label_, LV_OBJ_FLAG_HIDDEN);

    status_label_ = lv_label_create(status_bar_);
    lv_obj_set_width(status_label_, LV_HOR_RES);
    // 这条状态字和 top_bar_ 都挂在 y=0、都 16px 高，是完全重叠的。
    // 用 SCROLL_CIRCULAR 时文字会循环滚动扫过 WiFi 图标，叠成一团乱码
    // （以前 SetStatus 是空实现，这行从没被写过，所以没暴露）。
    // 改成 DOT：超长显示省略号，不滚，不抢 top_bar_ 的区域。
    lv_label_set_long_mode(status_label_, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(status_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(status_label_, Lang::Strings::INITIALIZING);
    lv_obj_align(status_label_, LV_ALIGN_CENTER, 0, 0);

    /* Content */
    content_ = lv_obj_create(container_);
    lv_obj_set_scrollbar_mode(content_, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_radius(content_, 0, 0);
    lv_obj_set_style_pad_all(content_, 0, 0);
    lv_obj_set_width(content_, LV_HOR_RES);
    lv_obj_set_flex_grow(content_, 1);
    lv_obj_set_flex_flow(content_, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_flex_main_place(content_, LV_FLEX_ALIGN_CENTER, 0);

    content_left_ = lv_obj_create(content_);
    lv_obj_set_size(content_left_, 32, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(content_left_, 0, 0);
    lv_obj_set_style_border_width(content_left_, 0, 0);

    emotion_label_ = lv_label_create(content_left_);
    lv_obj_set_style_text_font(emotion_label_, large_icon_font, 0);
    lv_label_set_text(emotion_label_, FONT_AWESOME_MICROCHIP_AI);
    lv_obj_center(emotion_label_);
    lv_obj_set_style_pad_top(emotion_label_, 8, 0);

    content_right_ = lv_obj_create(content_);
    lv_obj_set_size(content_right_, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(content_right_, 0, 0);
    lv_obj_set_style_border_width(content_right_, 0, 0);
    lv_obj_set_flex_grow(content_right_, 1);
    lv_obj_add_flag(content_right_, LV_OBJ_FLAG_HIDDEN);

    chat_message_label_ = lv_label_create(content_right_);
    lv_label_set_text(chat_message_label_, "");
    lv_label_set_long_mode(chat_message_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_text_align(chat_message_label_, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_width(chat_message_label_, width_ - 32);
    lv_obj_set_style_pad_top(chat_message_label_, 14, 0);

    // Start scrolling subtitle after a delay
    static lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_delay(&a, 1000);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_obj_set_style_anim(chat_message_label_, &a, LV_PART_MAIN);
    lv_obj_set_style_anim_duration(chat_message_label_, lv_anim_speed_clamped(60, 300, 60000), LV_PART_MAIN);

    low_battery_popup_ = lv_obj_create(screen);
    lv_obj_set_scrollbar_mode(low_battery_popup_, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_size(low_battery_popup_, LV_HOR_RES * 0.9, text_font->line_height * 2);
    lv_obj_align(low_battery_popup_, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(low_battery_popup_, lv_color_black(), 0);
    lv_obj_set_style_radius(low_battery_popup_, 10, 0);
    low_battery_label_ = lv_label_create(low_battery_popup_);
    lv_label_set_text(low_battery_label_, Lang::Strings::BATTERY_NEED_CHARGE);
    lv_obj_set_style_text_color(low_battery_label_, lv_color_white(), 0);
    lv_obj_center(low_battery_label_);
    lv_obj_add_flag(low_battery_popup_, LV_OBJ_FLAG_HIDDEN);

    /* 第 6 步：整屏大字时钟。挂 screen 而不是 container_，
       这样 ShowClock() 只要把普通界面藏起来，时钟就是整屏唯一的东西。
       拆成 时/冒号/分 三个 label，冒号才能单独闪。 */
    clock_row_ = lv_obj_create(screen);
    lv_obj_remove_style_all(clock_row_);
    lv_obj_set_size(clock_row_, LV_HOR_RES, 44);
    lv_obj_set_flex_flow(clock_row_, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(clock_row_, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(clock_row_, 2, 0);
    lv_obj_align(clock_row_, LV_ALIGN_TOP_MID, 0, 2);
    lv_obj_add_flag(clock_row_, LV_OBJ_FLAG_HIDDEN);

    clock_h_ = lv_label_create(clock_row_);
    lv_label_set_text(clock_h_, "--");
    lv_obj_set_style_text_font(clock_h_, &lv_font_montserrat_40, 0);

    clock_colon_ = lv_label_create(clock_row_);
    lv_label_set_text(clock_colon_, ":");
    lv_obj_set_style_text_font(clock_colon_, &lv_font_montserrat_40, 0);

    clock_m_ = lv_label_create(clock_row_);
    lv_label_set_text(clock_m_, "--");
    lv_obj_set_style_text_font(clock_m_, &lv_font_montserrat_40, 0);

    date_label_ = lv_label_create(screen);
    lv_label_set_text(date_label_, "");
    lv_obj_set_style_text_font(date_label_, text_font, 0);
    lv_obj_align(date_label_, LV_ALIGN_BOTTOM_MID, 0, -2);
    lv_obj_add_flag(date_label_, LV_OBJ_FLAG_HIDDEN);

    /* 7b：音乐界面。同样挂 screen，和正常界面、时钟三者互斥。
       128×64 分四段：第 0 页标题行、第 1-2 页歌名、第 3-6 页当前歌词。 */
    music_title_ = lv_label_create(screen);
    lv_label_set_text(music_title_, "");
    lv_obj_set_style_text_font(music_title_, text_font, 0);
    lv_obj_set_width(music_title_, width_ - 4);
    lv_obj_set_style_text_align(music_title_, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(music_title_, LV_LABEL_LONG_SCROLL_CIRCULAR);
    // 照官方 chat_message_label_ 的配置抄：延迟 1 秒再滚、无限循环、
    // 60 px/s 并 clamp 到 300-60000ms。之前只设了 SCROLL_CIRCULAR 没配动画，
    // 用的是 LVGL 默认参数，看着就是不滚。
    static lv_anim_t title_anim;
    lv_anim_init(&title_anim);
    lv_anim_set_delay(&title_anim, 1000);
    lv_anim_set_repeat_count(&title_anim, LV_ANIM_REPEAT_INFINITE);
    lv_obj_set_style_anim(music_title_, &title_anim, LV_PART_MAIN);
    lv_obj_set_style_anim_duration(music_title_, lv_anim_speed_clamped(60, 300, 60000), LV_PART_MAIN);
    lv_obj_align(music_title_, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_add_flag(music_title_, LV_OBJ_FLAG_HIDDEN);

    music_lyric_ = lv_label_create(screen);
    lv_label_set_text(music_lyric_, "");
    lv_obj_set_style_text_font(music_lyric_, text_font, 0);
    lv_obj_set_width(music_lyric_, width_ - 4);
    lv_obj_set_height(music_lyric_, height_ - 18);
    lv_obj_set_style_text_align(music_lyric_, LV_TEXT_ALIGN_CENTER, 0);
    // 歌词区占额头以下整块（46px 高），自动换行不滚动：
    // 128px 宽 14px 字体每行约 8 个汉字，3 行能放 24 个，够绝大部分歌词。
    // 滚动改成换行是因为歌词是给人读的，等它滚过去比直接看累。
    lv_label_set_long_mode(music_lyric_, LV_LABEL_LONG_WRAP);
    // 同上。anim 对象必须独立——lv_obj_set_style_anim 只存指针，
    // 两个 label 共用一个 static 会让后设置的覆盖前面的。
    lv_obj_align(music_lyric_, LV_ALIGN_TOP_MID, 0, 18);
    lv_obj_add_flag(music_lyric_, LV_OBJ_FLAG_HIDDEN);
}

void OledDisplay::SetupUI_128x32() {
    DisplayLockGuard lock(this);

    auto lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    auto text_font = lvgl_theme->text_font()->font();
    auto icon_font = lvgl_theme->icon_font()->font();
    auto large_icon_font = lvgl_theme->large_icon_font()->font();

    auto screen = lv_screen_active();
    lv_obj_set_style_text_font(screen, text_font, 0);

    /* Container */
    container_ = lv_obj_create(screen);
    lv_obj_set_size(container_, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_flex_flow(container_, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(container_, 0, 0);
    lv_obj_set_style_border_width(container_, 0, 0);
    lv_obj_set_style_pad_column(container_, 0, 0);

    /* Emotion label on the left side */
    content_ = lv_obj_create(container_);
    lv_obj_set_size(content_, 32, 32);
    lv_obj_set_style_pad_all(content_, 0, 0);
    lv_obj_set_style_border_width(content_, 0, 0);
    lv_obj_set_style_radius(content_, 0, 0);

    emotion_label_ = lv_label_create(content_);
    lv_obj_set_style_text_font(emotion_label_, large_icon_font, 0);
    lv_label_set_text(emotion_label_, FONT_AWESOME_MICROCHIP_AI);
    lv_obj_center(emotion_label_);

    /* Right side */
    side_bar_ = lv_obj_create(container_);
    lv_obj_set_size(side_bar_, width_ - 32, 32);
    lv_obj_set_flex_flow(side_bar_, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(side_bar_, 0, 0);
    lv_obj_set_style_border_width(side_bar_, 0, 0);
    lv_obj_set_style_radius(side_bar_, 0, 0);
    lv_obj_set_style_pad_row(side_bar_, 0, 0);

    /* Status bar */
    status_bar_ = lv_obj_create(side_bar_);
    lv_obj_set_size(status_bar_, width_ - 32, 16);
    lv_obj_set_style_radius(status_bar_, 0, 0);
    lv_obj_set_flex_flow(status_bar_, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(status_bar_, 0, 0);
    lv_obj_set_style_border_width(status_bar_, 0, 0);
    lv_obj_set_style_pad_column(status_bar_, 0, 0);

    status_label_ = lv_label_create(status_bar_);
    lv_obj_set_flex_grow(status_label_, 1);
    lv_obj_set_style_pad_left(status_label_, 2, 0);
    lv_label_set_text(status_label_, Lang::Strings::INITIALIZING);

    notification_label_ = lv_label_create(status_bar_);
    lv_obj_set_flex_grow(notification_label_, 1);
    lv_obj_set_style_pad_left(notification_label_, 2, 0);
    lv_label_set_text(notification_label_, "");
    lv_obj_add_flag(notification_label_, LV_OBJ_FLAG_HIDDEN);

    mute_label_ = lv_label_create(status_bar_);
    lv_label_set_text(mute_label_, "");
    lv_obj_set_style_text_font(mute_label_, icon_font, 0);

    network_label_ = lv_label_create(status_bar_);
    lv_label_set_text(network_label_, "");
    lv_obj_set_style_text_font(network_label_, icon_font, 0);

    battery_label_ = lv_label_create(status_bar_);
    lv_label_set_text(battery_label_, "");
    lv_obj_set_style_text_font(battery_label_, icon_font, 0);

    chat_message_label_ = lv_label_create(side_bar_);
    lv_obj_set_size(chat_message_label_, width_ - 32, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_left(chat_message_label_, 2, 0);
    lv_label_set_long_mode(chat_message_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_label_set_text(chat_message_label_, "");

    // Start scrolling subtitle after a delay
    static lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_delay(&a, 1000);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_obj_set_style_anim(chat_message_label_, &a, LV_PART_MAIN);
    lv_obj_set_style_anim_duration(chat_message_label_, lv_anim_speed_clamped(60, 300, 60000), LV_PART_MAIN);
}

void OledDisplay::SetEmotion(const char* emotion) {
    DisplayLockGuard lock(this);

    // 第 2 步：眼睛动画模式下，表情交给 RoboEyes；否则走原来的图标 label
    if (animated_emotion_mode_) {
        if (roboeyes_adapter_ != nullptr) {
            roboeyes_adapter_->SetEmotion(emotion);
        }
        return;
    }

    const char* utf8 = font_awesome_get_utf8(emotion);
    if (emotion_label_ == nullptr) {
        return;
    }
    if (utf8 != nullptr) {
        lv_label_set_text(emotion_label_, utf8);
    } else {
        lv_label_set_text(emotion_label_, FONT_AWESOME_NEUTRAL);
    }
}

void OledDisplay::SetTheme(Theme* theme) {
    DisplayLockGuard lock(this);

    auto lvgl_theme = static_cast<LvglTheme*>(theme);
    auto text_font = lvgl_theme->text_font()->font();

    auto screen = lv_screen_active();
    lv_obj_set_style_text_font(screen, text_font, 0);
}

// === 第 2 步：眼睛动画 ===
// RoboEyes 在 Begin() 里传了 panel_io_/panel_，所以它自己开一个 FreeRTOS 任务
// 直接调 esp_lcd_panel_draw_bitmap 刷屏，不依赖 LVGL 的定时器。这也是为什么
// 这里必须先 lvgl_port_stop()——两个渲染器同时写同一个 panel 会互相踩。

void OledDisplay::SetAnimatedEmotionMode(bool enable) {
    Display::SetAnimatedEmotionMode(enable);

    DisplayLockGuard lock(this);

    if (enable) {
        // 幂等：已经在动画模式就什么都不做。
        // v206 的日志里这组 "Stopping LVGL port" 每 5 秒刷一次，
        // 就是缺了这个判断，反复销毁重建 RoboEyes 任务。
        if (roboeyes_adapter_ != nullptr) {
            return;
        }

        ESP_LOGI(TAG, "Enabling animated emotion mode (eyes 0-47, subtitle 48-63)");
        // 分屏布局：上半 48 行给眼睛（眼睛任务直刷 panel），
        // 下半 16 行给字幕（LVGL 画）。
        //
        // 顺序很重要：先把官方容器藏掉、把字幕条建好，让 LVGL 把该重绘的
        // 都重绘完，再清屏、再起眼睛。反过来的话 LVGL 那次全屏重绘会落在
        // 眼睛已经画好的上半屏上，眼睛下一帧再画回来，就是一路黑闪。
        // 把所有官方控件一律藏掉，不看它们之前的状态。
        // HideClock() 里有 "if (!clock_visible_) return;" 的早退，
        // 而 clock_row_ 的可见状态并不总跟 clock_visible_ 同步
        // （创建时是隐藏的，但某些路径上会被单独放开），
        // 只靠 HideClock 会漏，这里直接兜底。
        if (container_ != nullptr) {
            lv_obj_add_flag(container_, LV_OBJ_FLAG_HIDDEN);
        }
        if (clock_row_ != nullptr) {
            lv_obj_add_flag(clock_row_, LV_OBJ_FLAG_HIDDEN);
        }
        if (date_label_ != nullptr) {
            lv_obj_add_flag(date_label_, LV_OBJ_FLAG_HIDDEN);
        }
        if (status_bar_ != nullptr) {
            lv_obj_add_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
        }
        clock_visible_ = false;
        if (emotion_label_ != nullptr) {
            lv_obj_add_flag(emotion_label_, LV_OBJ_FLAG_HIDDEN);
        }

        // 清屏：把 LVGL 留下的时钟、状态栏、官方容器一次性抹掉。
        // 不清的话眼睛只发有变化的页，那些残留要等眼睛晃过去才被覆盖，
        // 表现就是"脑门上挂着一个从没刷新过的时间"。
        if (panel_ != nullptr) {
            static const uint8_t kBlank[128 * 64 / 8] = {0};
            esp_err_t blank_err = esp_lcd_panel_draw_bitmap(panel_, 0, 0, width(), height(), kBlank);
            if (blank_err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to clear panel: %s", esp_err_to_name(blank_err));
            }
        }

        roboeyes_adapter_ = std::make_unique<RoboEyesAdapter>();
        // 高度传 height() - 16：眼睛只画上半 48 行。
        if (!roboeyes_adapter_->Begin(content_left_, width(), height(), 30, panel_io_, panel_)) {
            ESP_LOGE(TAG, "Failed to initialize RoboEyesAdapter");
            roboeyes_adapter_.reset();
            animated_emotion_mode_ = false;
            return;
        }
    } else {
        if (roboeyes_adapter_ == nullptr) {
            return;
        }

        ESP_LOGI(TAG, "Disabling animated emotion mode");
        // 先确认任务能停下来，再决定是销毁还是泄漏。
        // 直接 reset() 的后果：析构等不到任务退出 → 对象被 delete →
        // 任务继续访问已释放的 this->bit_buffer_/eyes_obj_ → LoadProhibited。
        // 停不下来时宁可泄漏这一份 adapter，也不能让任务踩空。
        if (!roboeyes_adapter_->StopPanelTask(2000)) {
            ESP_LOGE(TAG, "panel task refused to stop, leaking adapter to avoid use-after-free");
            roboeyes_adapter_.release();
        } else {
            roboeyes_adapter_.reset();
        }

        // 分屏模式下 LVGL 一直在跑，没有 stop 过，所以这里不 resume。
        if (emotion_label_ != nullptr) {
            lv_obj_remove_flag(emotion_label_, LV_OBJ_FLAG_HIDDEN);
        }
        // 恢复官方控件，藏起分屏字幕。
        // status_bar_ 挂的是 screen 不是 container_，
        // 不单独放开的话，退出眼睛后状态栏会一直不回来。
        if (container_ != nullptr) {
            lv_obj_remove_flag(container_, LV_OBJ_FLAG_HIDDEN);
        }
        if (status_bar_ != nullptr) {
            lv_obj_remove_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

void OledDisplay::SetEmotionDirection(int direction) {
    // RoboEyes 自己管理眼球的游走，这个接口留着给以后"看向某个方向"用
    (void)direction;
}

void OledDisplay::UpdateAnimatedEmotion() {
    // RoboEyes 的动画由它自己的任务驱动，这里不需要推
}


// === 第 6 步：整屏大字时钟 ===
// 睡眠态显示。和 container_（状态栏 + 表情 + 字幕）互斥：
// 显示时钟就把 container_ 藏起来，退出时钟再放回来。

void OledDisplay::ShowClock() {
    DisplayLockGuard lock(this);
    if (!lock.locked()) {
        return;
    }
    if (clock_row_ == nullptr) {
        return;
    }

    // 幂等：状态没变就只更新文本。
    // 之前每秒都无条件 add_flag/remove_flag，每次都会触发整屏重绘，
    // 40px 字体的中文日期重排一次不便宜，持锁时间被拉长。
    if (clock_visible_) {
        UpdateClockText();
        return;
    }

    // 收起普通界面。status_bar_ 挂的是 screen 不是 container_，
    // 只藏 container_ 会留下"待机"那行字压在时钟上，两个都要藏。
    if (container_ != nullptr) {
        lv_obj_add_flag(container_, LV_OBJ_FLAG_HIDDEN);
    }
    if (status_bar_ != nullptr) {
        lv_obj_add_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
    }

    lv_obj_remove_flag(clock_row_, LV_OBJ_FLAG_HIDDEN);
    if (date_label_ != nullptr) {
        lv_obj_remove_flag(date_label_, LV_OBJ_FLAG_HIDDEN);
    }

    clock_visible_ = true;
    UpdateClockText();
}

void OledDisplay::HideClock() {
    DisplayLockGuard lock(this);
    if (!lock.locked()) {
        return;
    }
    if (!clock_visible_) {
        return;
    }
    clock_visible_ = false;
    if (clock_row_ != nullptr) {
        lv_obj_add_flag(clock_row_, LV_OBJ_FLAG_HIDDEN);
    }
    if (date_label_ != nullptr) {
        lv_obj_add_flag(date_label_, LV_OBJ_FLAG_HIDDEN);
    }
    if (container_ != nullptr) {
        lv_obj_remove_flag(container_, LV_OBJ_FLAG_HIDDEN);
    }
    if (status_bar_ != nullptr) {
        lv_obj_remove_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
    }
}

// === 7b：音乐界面 ===
// on=true：藏起正常界面与时钟，只留三个音乐 label。
// on=false：反过来。title/lyric 传 nullptr 表示"这次不改这一行"，
// 因为是分三条 sentence_start 陆续到的（先 play_music，再歌名，再歌词）。
void OledDisplay::SetMusicMode(bool on, const char* title, const char* lyric) {
    DisplayLockGuard lock(this);
    if (!lock.locked()) {
        return;
    }

    if (!on) {
        if (!music_visible_) {
            return;
        }
        music_visible_ = false;

        if (music_title_ != nullptr) {
            lv_obj_add_flag(music_title_, LV_OBJ_FLAG_HIDDEN);
        }
        if (music_lyric_ != nullptr) {
            lv_obj_add_flag(music_lyric_, LV_OBJ_FLAG_HIDDEN);
        }
        if (container_ != nullptr) {
            lv_obj_remove_flag(container_, LV_OBJ_FLAG_HIDDEN);
        }
        if (status_bar_ != nullptr) {
            lv_obj_remove_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }

    // 第一次进来：把别的界面全藏掉。不看之前的可见状态，一律藏——
    // 内部标志和控件实际可见状态不同步时，靠"之前是什么就恢复什么"会漏。
    if (!music_visible_) {
        music_visible_ = true;
        if (container_ != nullptr) {
            lv_obj_add_flag(container_, LV_OBJ_FLAG_HIDDEN);
        }
        if (status_bar_ != nullptr) {
            lv_obj_add_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
        }
        if (clock_row_ != nullptr) {
            lv_obj_add_flag(clock_row_, LV_OBJ_FLAG_HIDDEN);
        }
        if (date_label_ != nullptr) {
            lv_obj_add_flag(date_label_, LV_OBJ_FLAG_HIDDEN);
        }
        clock_visible_ = false;

        if (music_title_ != nullptr) {
            lv_obj_remove_flag(music_title_, LV_OBJ_FLAG_HIDDEN);
        }
        if (music_lyric_ != nullptr) {
            lv_obj_remove_flag(music_lyric_, LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (title != nullptr && music_title_ != nullptr) {
        // 上层传下来的 title 已经剥掉书名号了（识别时是掐头去尾取子串），
        // 显示时补回去。
        // 直接写中文字面量。之前用 "ã" 这种字节转义，
        // 屏幕上出来是「ü逐梦ü」——第三个字节没进到字符串里，
        // LVGL 拿到半截 UTF-8 就只能画替代字形。源文件本身是 UTF-8，
        // 直接写字面量最稳。
        std::string with_marks = "《";
        with_marks += title;
        with_marks += "》";
        lv_label_set_text(music_title_, with_marks.c_str());
    }
    if (lyric != nullptr && music_lyric_ != nullptr) {
        lv_label_set_text(music_lyric_, lyric);
    }
}

void OledDisplay::UpdateClockText() {
    if (clock_h_ == nullptr || clock_m_ == nullptr) {
        return;
    }

    time_t now = time(nullptr);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    // NTP / OTA 时间都还没到位时，别显示 1970 年
    if (timeinfo.tm_year < 2025 - 1900) {
        lv_label_set_text(clock_h_, "--");
        lv_label_set_text(clock_m_, "--");
        lv_obj_set_style_text_opa(clock_colon_, LV_OPA_COVER, 0);
        if (date_label_ != nullptr) {
            lv_label_set_text(date_label_, "");
        }
        return;
    }

    char buf[8];
    snprintf(buf, sizeof(buf), "%02d", timeinfo.tm_hour);
    lv_label_set_text(clock_h_, buf);
    snprintf(buf, sizeof(buf), "%02d", timeinfo.tm_min);
    lv_label_set_text(clock_m_, buf);

    // 冒号一秒亮一秒灭。用透明度而不是 HIDDEN：flex 会把隐藏对象
    // 从排列里剔除，20:40 会瞬间挤成 2040 再弹回来。
    if (clock_colon_ != nullptr) {
        lv_obj_set_style_text_opa(clock_colon_,
            (timeinfo.tm_sec & 1) ? LV_OPA_TRANSP : LV_OPA_COVER, 0);
    }

    if (date_label_ != nullptr) {
        static const char* kWeekday[] = {"周日", "周一", "周二", "周三", "周四", "周五", "周六"};
        char date_str[32];
        snprintf(date_str, sizeof(date_str), "%02d/%02d %s",
                 timeinfo.tm_mon + 1, timeinfo.tm_mday,
                 kWeekday[timeinfo.tm_wday]);
        lv_label_set_text(date_label_, date_str);
    }
}
