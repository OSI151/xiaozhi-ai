#include "rgb_effects.h"

#include "audio/playback_level.h"

#include <cstring>
#include <cstdlib>

// ---------------------------------------------------------------- 小工具

namespace {

// 正弦查表：-127..127，一个周期 256 步。
// 用表不用 sinf 是为了让每帧的开销可预测——14 个效果里有一半要算相位，
// 每帧 7 次 libm 调用在低优先级任务里抖动会累积成肉眼可见的顿。
const int16_t kSin256[256] = {
       0,    3,    6,    9,   12,   16,   19,   22,   25,   28,   31,   34,   37,   40,   43,   46,
      49,   51,   54,   57,   60,   63,   65,   68,   71,   73,   76,   78,   81,   83,   85,   88,
      90,   92,   94,   96,   98,  100,  102,  104,  106,  107,  109,  111,  112,  113,  115,  116,
     117,  118,  119,  120,  121,  122,  122,  123,  124,  124,  125,  125,  125,  126,  126,  126,
     126,  126,  126,  126,  125,  125,  125,  124,  124,  123,  122,  122,  121,  120,  119,  118,
     117,  116,  115,  113,  112,  111,  109,  107,  106,  104,  102,  100,   98,   96,   94,   92,
      90,   88,   85,   83,   81,   78,   76,   73,   71,   68,   65,   63,   60,   57,   54,   51,
      49,   46,   43,   40,   37,   34,   31,   28,   25,   22,   19,   16,   12,    9,    6,    3,
       0,   -3,   -6,   -9,  -12,  -16,  -19,  -22,  -25,  -28,  -31,  -34,  -37,  -40,  -43,  -46,
     -49,  -51,  -54,  -57,  -60,  -63,  -65,  -68,  -71,  -73,  -76,  -78,  -81,  -83,  -85,  -88,
     -90,  -92,  -94,  -96,  -98, -100, -102, -104, -106, -107, -109, -111, -112, -113, -115, -116,
    -117, -118, -119, -120, -121, -122, -122, -123, -124, -124, -125, -125, -125, -126, -126, -126,
    -126, -126, -126, -126, -125, -125, -125, -124, -124, -123, -122, -122, -121, -120, -119, -118,
    -117, -116, -115, -113, -112, -111, -109, -107, -106, -104, -102, -100,  -98,  -96,  -94,  -92,
     -90,  -88,  -85,  -83,  -81,  -78,  -76,  -73,  -71,  -68,  -65,  -63,  -60,  -57,  -54,  -51,
     -49,  -46,  -43,  -40,  -37,  -34,  -31,  -28,  -25,  -22,  -19,  -16,  -12,   -9,   -6,   -3,
};

// 相位 0..255 的正弦，返回 -127..127
inline int Sin8(uint32_t phase) { return kSin256[phase & 0xFF]; }
// 相位 0..255，返回 0..255（正弦抬到正值区间）
inline uint8_t Sin8u(uint32_t phase) { return (uint8_t)((Sin8(phase) + 127)); }

inline uint8_t Scale8(uint8_t v, uint8_t f) {
    return (uint8_t)(((uint16_t)v * f) / 255);
}

inline StripColor Scale(StripColor c, uint8_t f) {
    return StripColor{Scale8(c.r, f), Scale8(c.g, f), Scale8(c.b, f)};
}

inline StripColor Blend(StripColor a, StripColor b, uint8_t f) {
    // f=0 全 a，f=255 全 b
    const uint16_t g = 255 - f;
    return StripColor{
        (uint8_t)((a.r * g + b.r * f) / 255),
        (uint8_t)((a.g * g + b.g * f) / 255),
        (uint8_t)((a.b * g + b.b * f) / 255),
    };
}

// 色相 0..255 一圈。s/v 0..255。
StripColor Hsv(uint8_t h, uint8_t s, uint8_t v) {
    if (s == 0) {
        return StripColor{v, v, v};
    }
    const uint8_t region = (uint8_t)(h / 43);          // 0..5
    const uint8_t rem = (uint8_t)((h - region * 43) * 6);  // 0..255
    const uint8_t p = (uint8_t)(((uint16_t)v * (255 - s)) / 255);
    const uint8_t q = (uint8_t)(((uint16_t)v * (255 - ((uint16_t)s * rem) / 255)) / 255);
    const uint8_t t = (uint8_t)(((uint16_t)v * (255 - ((uint16_t)s * (255 - rem)) / 255)) / 255);
    switch (region) {
        case 0: return StripColor{v, t, p};
        case 1: return StripColor{q, v, p};
        case 2: return StripColor{p, v, t};
        case 3: return StripColor{p, q, v};
        case 4: return StripColor{t, p, v};
        default: return StripColor{v, p, q};
    }
}

// speed(0..255) → 周期毫秒。0 最慢、255 最快。
inline uint32_t PeriodMs(uint8_t speed, uint32_t slow_ms, uint32_t fast_ms) {
    const uint32_t span = slow_ms - fast_ms;
    return fast_ms + (span * (uint32_t)(255 - speed)) / 255u;
}

// 无状态伪随机：给定种子返回 0..255。效果里要靠它做"每周期换一次"，
// 不能用 rand()——那会把全局状态搅乱，也不可复现。
inline uint8_t Hash8(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352dUL;
    x ^= x >> 15;
    x *= 0x846ca68bUL;
    x ^= x >> 16;
    return (uint8_t)(x & 0xFF);
}

// 每个效果至少要有一组颜色；没给就用白色，避免全黑看起来像故障。
inline StripColor FirstOrDefault(const StripParams& p) {
    if (p.color_count == 0) {
        return StripColor{255, 255, 255};
    }
    return p.ColorOf(0);
}

// ---------------------------------------------------------------- 效果实现

void RenderSolid(const StripParams& p, uint32_t /*t*/, uint16_t count, StripColor* out) {
    const StripColor c = FirstOrDefault(p);
    for (uint16_t i = 0; i < count; ++i) {
        out[i] = c;
    }
}

void RenderBreathe(const StripParams& p, uint32_t t, uint16_t count, StripColor* out) {
    const StripColor c = FirstOrDefault(p);
    const uint32_t period = PeriodMs(p.speed, 5000, 400);
    // intensity 抬高最低亮度：128 时从 0 起，255 时几乎不灭
    const uint8_t floor_v = (uint8_t)(p.intensity > 128 ? (p.intensity - 128) * 2 : 0);
    const uint8_t s = Sin8u((uint32_t)((uint64_t)t * 256 / period));
    const uint8_t k = (uint8_t)(floor_v + (uint8_t)(((uint16_t)(255 - floor_v) * s) / 255));
    const StripColor v = Scale(c, k);
    for (uint16_t i = 0; i < count; ++i) {
        out[i] = v;
    }
}

void RenderPulse(const StripParams& p, uint32_t t, uint16_t count, StripColor* out) {
    const StripColor c = FirstOrDefault(p);
    const uint32_t period = PeriodMs(p.speed, 3000, 300);
    // 急升慢落：把正弦平方，峰值附近锐利、尾部拖长
    const uint8_t s = Sin8u((uint32_t)((uint64_t)t * 256 / period));
    uint16_t k = (uint16_t)s * s / 255;
    // intensity 越大拖尾越短
    if (p.intensity > 160) {
        k = (uint16_t)(k * k / 255);
    }
    const StripColor v = Scale(c, (uint8_t)k);
    for (uint16_t i = 0; i < count; ++i) {
        out[i] = v;
    }
}

void RenderRainbow(const StripParams& p, uint32_t t, uint16_t count, StripColor* out) {
    const uint32_t period = PeriodMs(p.speed, 8000, 800);
    const uint8_t hue = (uint8_t)(((uint64_t)t * 256 / period) & 0xFF);
    // intensity 控制饱和度。不能用它调 v：v 会被 Dim() 的硬上限截平，
    // 而且 160 + intensity/2 在 intensity 大时会溢出 uint8（287 -> 31），
    // 表现就是"越调越暗"。
    const uint8_t sat = (uint8_t)(128 + p.intensity / 2);
    const StripColor c = Hsv(hue, sat, 255);
    for (uint16_t i = 0; i < count; ++i) {
        out[i] = c;
    }
}

void RenderRainbowFlow(const StripParams& p, uint32_t t, uint16_t count, StripColor* out) {
    const uint32_t period = PeriodMs(p.speed, 6000, 600);
    const uint8_t base = (uint8_t)(((uint64_t)t * 256 / period) & 0xFF);
    // c1 控制相邻灯珠的色相间隔：0 时整条一色，255 时几乎绕一圈
    const uint8_t step = (uint8_t)(8 + p.c1 / 4);
    const uint8_t sat = (uint8_t)(128 + p.intensity / 2);
    for (uint16_t i = 0; i < count; ++i) {
        out[i] = Hsv((uint8_t)(base + i * step), sat, 255);
    }
}

void RenderGradient(const StripParams& p, uint32_t t, uint16_t count, StripColor* out) {
    const StripColor a = FirstOrDefault(p);
    const StripColor b = p.color_count >= 2 ? p.ColorOf(1) : a;
    if (count == 0) {
        return;
    }
    // c2 = 0 时静止；>0 时整条渐变来回漂移，看起来像光在爬
    uint8_t basis = 0;
    if (p.c2 > 0) {
        const uint32_t period = PeriodMs(p.speed, 8000, 1000);
        basis = (uint8_t)(((uint64_t)t * 256 / period) & 0xFF);
    }
    for (uint16_t i = 0; i < count; ++i) {
        uint8_t f = count > 1 ? (uint8_t)((uint32_t)i * 255 / (count - 1)) : 0;
        f = (uint8_t)(f + basis);
        out[i] = Blend(a, b, f);
    }
}

void RenderColorWipe(const StripParams& p, uint32_t t, uint16_t count, StripColor* out) {
    if (count == 0) {
        return;
    }
    const uint8_t n = p.color_count ? p.color_count : 1;
    const uint32_t period = PeriodMs(p.speed, 4000, 400);
    // 整条走一遍算一个周期；每颗按"当前扫到第几颗"取色
    const uint32_t cell_ms = period / (count > 0 ? count : 1);
    if (cell_ms == 0) {
        return;
    }
    const uint32_t head = (t / cell_ms) % (uint32_t)(count * n + count);
    for (uint16_t i = 0; i < count; ++i) {
        const uint32_t pos = (uint32_t)i;
        out[i] = head >= pos ? p.ColorOf((uint8_t)((head - pos) / count % n)) : StripColor{0, 0, 0};
    }
}

void RenderChase(const StripParams& p, uint32_t t, uint16_t count, StripColor* out) {
    if (count == 0) {
        return;
    }
    const StripColor c = FirstOrDefault(p);
    const uint32_t period = PeriodMs(p.speed, 2000, 150);
    // c1 控制尾迹长度：0 只有单点，255 几乎拖满整条
    const uint16_t tail = (uint16_t)((uint32_t)p.c1 * (count - 1) / 255);
    const uint32_t step_ms = period / (count > 0 ? count : 1);
    if (step_ms == 0) {
        return;
    }
    const uint16_t head = (uint16_t)((t / step_ms) % (count * 2));
    const int pos = head < count ? (int)head : (int)(count * 2 - 1 - head);
    for (uint16_t i = 0; i < count; ++i) {
        const int d = pos - (int)i;
        if (d < 0) {
            out[i] = StripColor{0, 0, 0};
        } else if (d == 0) {
            out[i] = c;
        } else if ((uint16_t)d <= tail) {
            const uint8_t f = (uint8_t)(255 - (d * 255) / (tail + 1));
            out[i] = Scale(c, f);
        } else {
            out[i] = StripColor{0, 0, 0};
        }
    }
}

void RenderBlink(const StripParams& p, uint32_t t, uint16_t count, StripColor* out) {
    const uint32_t period = PeriodMs(p.speed, 2000, 200);
    // c2 当"闪几次"用，0 表示一直闪。闪满次数后停在常亮，
    // 比停在黑更合理：灯突然全灭容易被当成故障。
    const uint32_t cycle = period ? (t / period) : 0;
    const StripColor c = FirstOrDefault(p);
    const uint8_t n = p.color_count ? p.color_count : 1;
    const StripColor cur = p.ColorOf((uint8_t)(cycle % n));
    // intensity 控制占空比：越大亮的时间占比越长
    const uint32_t on_ms = period * (uint32_t)p.intensity / 255;
    const bool on = (t % period) < on_ms;
    const bool done = (p.c2 > 0) && (cycle >= p.c2);
    const StripColor v = done ? c : (on ? cur : StripColor{0, 0, 0});
    for (uint16_t i = 0; i < count; ++i) {
        out[i] = v;
    }
}

void RenderMeteor(const StripParams& p, uint32_t t, uint16_t count, StripColor* out) {
    if (count == 0) {
        return;
    }
    const StripColor c = FirstOrDefault(p);
    const uint32_t period = PeriodMs(p.speed, 2500, 250);
    const uint16_t tail = (uint16_t)(1 + (uint32_t)p.c1 * (count - 1) / 255);
    const uint32_t step_ms = period / (count > 0 ? count : 1);
    if (step_ms == 0) {
        return;
    }
    const uint16_t head = (uint16_t)((t / step_ms) % count);
    for (uint16_t i = 0; i < count; ++i) {
        const int d = (int)head - (int)i;
        if (d >= 0 && (uint16_t)d < tail) {
            const uint8_t f = (uint8_t)(255 - (d * 255) / tail);
            out[i] = Scale(c, f);
        } else {
            out[i] = StripColor{0, 0, 0};
        }
    }
}

void RenderLightning(const StripParams& p, uint32_t t, uint16_t count, StripColor* out) {
    const StripColor c = FirstOrDefault(p);
    const uint32_t period = PeriodMs(p.speed, 1200, 120);
    const uint32_t cycle = period ? (t / period) : 0;
    const uint32_t phase = period ? (t % period) : 0;
    const uint8_t r = Hash8(cycle * 2654435761u);
    // 一个周期里只有前 1/6 是亮的，其余暗；闪的强度随机
    const bool on = phase < (period / 6) && (r > 60);
    const uint8_t v = on ? (uint8_t)(200 + (r % 56)) : 0;
    const StripColor s = Scale(c, v);
    for (uint16_t i = 0; i < count; ++i) {
        out[i] = s;
    }
}

void RenderWave(const StripParams& p, uint32_t t, uint16_t count, StripColor* out) {
    const StripColor c = FirstOrDefault(p);
    const uint32_t period = PeriodMs(p.speed, 4000, 400);
    if (count == 0) {
        return;
    }
    const uint8_t span = (uint8_t)(32 + p.c1 / 2);   // 波长：越大起伏越密
    const uint8_t base = (uint8_t)(((uint64_t)t * 256 / period) & 0xFF);
    for (uint16_t i = 0; i < count; ++i) {
        const uint8_t k = Sin8u((uint8_t)(base + i * span));
        out[i] = Scale(c, k);
    }
}

void RenderSnow(const StripParams& p, uint32_t t, uint16_t count, StripColor* out) {
    const StripColor c = FirstOrDefault(p);
    const uint32_t period = PeriodMs(p.speed, 1500, 150);
    const uint32_t cycle = period ? (t / period) : 0;
    for (uint16_t i = 0; i < count; ++i) {
        const uint8_t r = Hash8(cycle * 31u + i * 97u);
        // intensity 控制"落雪密度"：越大同时亮的颗数越多
        out[i] = (r < p.intensity) ? c : StripColor{0, 0, 0};
    }
}

void RenderRacer(const StripParams& p, uint32_t t, uint16_t count, StripColor* out) {
    if (count == 0) {
        return;
    }
    const StripColor c = FirstOrDefault(p);
    const uint32_t period = PeriodMs(p.speed, 800, 80);
    const uint32_t step_ms = period / (count > 0 ? count : 1);
    if (step_ms == 0) {
        return;
    }
    const uint16_t head = (uint16_t)((t / step_ms) % count);
    // 两个头，相隔半条
    const uint16_t head2 = (uint16_t)((head + count / 2) % count);
    for (uint16_t i = 0; i < count; ++i) {
        out[i] = (i == head || i == head2) ? c : Scale(c, 24);
    }
}

// 语音律动：没有声音就没有灯。
// 音量包络来自播放通道（audio_service.cc 把 PCM 交给功放之前算的 RMS），
// 所以只有设备真的在出声时它才亮；对着麦克风说话不算。
void RenderVoice(const StripParams& p, uint32_t /*t*/, uint16_t count, StripColor* out) {
    if (count == 0) {
        return;
    }
    // 快升慢落。音频一帧 60ms、灯一帧 20ms，同一个音量值会被读到三次；
    // 衰减故意放在这一侧，拖尾长短才能按效果参数单独调。
    static uint8_t s_level = 0;
    const uint8_t raw = GetPlaybackLevel();
    const uint8_t decay = (uint8_t)(2 + p.speed / 32);   // 2..9，speed 越大退得越快
    if (raw > s_level) {
        s_level = raw;
    } else {
        s_level = (s_level > decay) ? (uint8_t)(s_level - decay) : 0;
    }
    if (s_level < 6) {
        s_level = 0;   // 底噪以下直接关，免得静音时星星点点地亮
    }

    const StripColor c = FirstOrDefault(p);
    const int center = (count - 1) / 2;
    const int maxd = center > 0 ? center : 1;
    // 扩散半径：音量 × 灵敏度，满值正好推到最外一颗
    const int gain = 128 + p.intensity / 2;          // 128..255
    const int radius = (int)s_level * gain / 255;    // 0..255
    for (uint16_t i = 0; i < count; ++i) {
        const int d = abs((int)i - center);
        const int dd = d * 255 / maxd;
        const int v = radius - dd;
        out[i] = (v <= 0) ? StripColor{0, 0, 0} : Scale(c, (uint8_t)(v > 255 ? 255 : v));
    }
}

}  // namespace

// ---------------------------------------------------------------- 效果表

const EffectDef kEffects[kFxCount] = {
    // name        letter mask                                            speed 说明        intensity 说明    c1 说明        c2 说明           min max render
    // 常亮没有字母：文档里常亮是直接写 RGB 三元组，不经过字母。
    {"常亮",       0,     kParamIntensity,                                  "",               "亮度下限",       "",            "",               1,  1, RenderSolid},
    {"呼吸",       'W',   kParamSpeed | kParamIntensity,                    "呼吸周期",       "最低亮度",       "",            "",               1,  1, RenderBreathe},
    {"脉冲",       'U',   kParamSpeed | kParamIntensity,                    "脉冲周期",       "拖尾长度",       "",            "",               1,  1, RenderPulse},
    {"彩虹",       'R',   kParamSpeed | kParamIntensity,                    "转一圈时间",     "亮度",           "",            "",               0,  0, RenderRainbow},
    {"彩虹流水",   'P',   kParamSpeed | kParamIntensity | kParamC1,         "流动速度",       "亮度",           "色相间隔",    "",               0,  0, RenderRainbowFlow},
    {"渐变",       'G',   kParamSpeed | kParamC2,                           "漂移周期",       "",               "",            "漂移开关",       2,  2, RenderGradient},
    {"多色流水",   'C',   kParamSpeed,                                      "扫过速度",       "",               "",            "",               2,  6, RenderColorWipe},
    {"跑马灯",     'S',   kParamSpeed | kParamC1,                           "跑动速度",       "",               "尾迹长度",    "",               1,  1, RenderChase},
    {"闪烁",       'F',   kParamSpeed | kParamIntensity | kParamC2,         "闪烁频率",       "亮灯占比",       "",            "闪几次",         1,  6, RenderBlink},
    {"流星",       'T',   kParamSpeed | kParamC1,                           "下落速度",       "",               "尾巴长度",    "",               1,  1, RenderMeteor},
    {"闪电",       'L',   kParamSpeed,                                      "闪动频率",       "",               "",            "",               1,  1, RenderLightning},
    {"波浪",       'V',   kParamSpeed | kParamC1,                           "波浪速度",       "",               "波长",        "",               1,  1, RenderWave},
    {"雪花",       'N',   kParamSpeed | kParamIntensity,                    "换图频率",       "密度",           "",            "",               1,  1, RenderSnow},
    {"赛车",       'K',   kParamSpeed,                                      "跑动速度",       "",               "",            "",               1,  1, RenderRacer},
    {"语音律动",   'B',   kParamSpeed | kParamIntensity,                    "响应速度",       "扩散范围",       "",            "",               1,  1, RenderVoice},
};

uint8_t FindEffectByLetter(char letter) {
    const char up = (char)((letter >= 'a' && letter <= 'z') ? letter - 32 : letter);
    // M = 多色闪烁，和 F 走同一支渲染（颜色多于一组时自然逐色闪），
    // 表里一个效果只能有一个字母，这样两个字母都能用。
    if (up == 'M') {
        return kFxBlink;
    }
    for (uint8_t i = 0; i < kFxCount; ++i) {
        if (kEffects[i].letter != 0 && kEffects[i].letter == up) {
            return i;
        }
    }
    return kFxCount;
}

uint8_t FindEffectByName(const char* name) {
    if (name == nullptr) {
        return kFxCount;
    }
    for (uint8_t i = 0; i < kFxCount; ++i) {
        if (strcmp(kEffects[i].name, name) == 0) {
            return i;
        }
    }
    return kFxCount;
}
