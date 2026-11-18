#ifndef _RGB_EFFECTS_H_
#define _RGB_EFFECTS_H_

#include <cstdint>

// WS2812 灯效表。照 WLED 的 FX 表思路做：效果是"一行表项 + 一支渲染回调"，
// 参数（速度/强度/c1/c2/颜色）统一放在 StripParams 里，Web 端按表项的 mask
// 决定显示哪几个滑杆。加新效果只需要动这个文件，UI 和保存逻辑不用改。
//
// 和 WLED 的差别：我们只驱动一条 7 颗的灯带，没有 segment、调色板、混合模式，
// 所以参数压到 4 个滑杆（speed / intensity / c1 / c2）+ 最多 6 组颜色。

struct StripColor {
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
};

// 渲染参数。单位统一用 0-255，和 WLED 的 segment 字段一致，
// 具体含义由效果自己解释（见 kEffects 里的 param_hint）。
struct StripParams {
    uint8_t effect = 0;      // EffectId
    uint8_t speed = 128;     // 越大越快
    uint8_t intensity = 128; // 幅度/数量
    uint8_t c1 = 128;        // 效果自定义 1（尾长、色相偏移……）
    uint8_t c2 = 128;        // 效果自定义 2
    StripColor colors[6];
    uint8_t color_count = 0;

    StripColor ColorOf(uint8_t i) const {
        return i < 6 ? colors[i] : StripColor{};
    }
};

enum EffectId : uint8_t {
    kFxSolid = 0,     // 纯色常亮
    kFxBreathe,       // 呼吸
    kFxPulse,         // 脉冲（急升慢落）
    kFxRainbow,       // 彩虹（整条同色循环）
    kFxRainbowFlow,   // 彩虹流水
    kFxGradient,      // 双色渐变
    kFxColorWipe,     // 多色流水（逐颗推进）
    kFxChase,         // 跑马灯（单点 + 尾迹）
    kFxBlink,         // 闪烁（F<n>）
    kFxMeteor,        // 流星
    kFxLightning,     // 闪电（占位实现）
    kFxWave,          // 波浪（占位实现）
    kFxSnow,          // 雪花（占位实现）
    kFxRacer,         // 赛车（占位实现）
    kFxVoice,         // 语音律动（跟着播放音量从中间往外扩散）
    kFxCount,
};

// 参数掩码：某位为 1 表示该效果会用到这个滑杆，Web 端据此显示/隐藏。
static constexpr uint8_t kParamSpeed = 1 << 0;
static constexpr uint8_t kParamIntensity = 1 << 1;
static constexpr uint8_t kParamC1 = 1 << 2;
static constexpr uint8_t kParamC2 = 1 << 3;

struct EffectDef {
    const char* name;     // 中文名，给 Web 和语音用
    char letter;          // gcode 字母；0 表示只能通过参数调用
    uint8_t param_mask;   // 用哪些滑杆
    const char* p1;       // speed 的人话说明
    const char* p2;       // intensity 的人话说明
    const char* p3;       // c1 的人话说明
    const char* p4;       // c2 的人话说明
    uint8_t color_min;    // 至少要几组颜色
    uint8_t color_max;    // 最多接受几组颜色
    // 每帧算一帧像素。out 长度 = count。t_ms 是绝对毫秒时间戳，
    // 相位必须由它算，不能靠"第几帧"累加——帧间隔抖动一下画面就会顿。
    void (*render)(const StripParams& p, uint32_t t_ms, uint16_t count, StripColor* out);
};

extern const EffectDef kEffects[kFxCount];

// 按 gcode 字母查效果，找不到返回 kFxCount。
uint8_t FindEffectByLetter(char letter);
// 按中文名或英文名查效果（语音/Web 用），找不到返回 kFxCount。
uint8_t FindEffectByName(const char* name);

#endif  // _RGB_EFFECTS_H_
