#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <esp_http_server.h>
#include <string>
#include <functional>
#include "peripherals/vl6180x.h"
#include <vector>

class WebServer {
public:
    WebServer();
    ~WebServer();

    bool Start(int port = 80);
    void Stop();

    // 注册电机控制回调函数 - 使用 std::function 支持 lambda
    void SetMotorControlCallback(std::function<void(int direction, int speed)> callback);
    // 由外部调用以触发电机控制（封装私有回调）
    void InvokeMotorControl(int direction, int speed);

    // 注册表情设置回调函数
    void SetEmotionCallback(std::function<void(const char* emotion)> callback);
    // 设置表情（封装私有回调）
    void SetEmotion(const char* emotion);

    // 电机动作配置回调
    struct MotorActionConfig {
        int forward_duration_ms = 5000;
        int backward_duration_ms = 5000;
        int left_turn_duration_ms = 600;
        int right_turn_duration_ms = 600;
        int spin_duration_ms = 2500;
        int wiggle_duration_ms = 600;
        int dance_duration_ms = 1500;
        int quick_forward_duration_ms = 5000;
        int quick_backward_duration_ms = 5000;
        int default_speed_percent = 100;
        // 跟随模式两个距离阈值，跟 Application::MotorActionConfig 一一对应
        int follow_far_mm = 100;
        int follow_near_mm = 50;
        int follow_speed = 55;
        int patrol_speed = 55;
        int th_blocked_mm = 80;
        int th_blocked_exit_mm = 100;
        int th_near_mm = 120;
        int th_near_exit_mm = 140;
    };

    void SetMotorActionConfigCallback(std::function<MotorActionConfig()> get_callback,
                                     std::function<void(const MotorActionConfig&)> set_callback);

    // 测距回调。Web 页面一秒轮询一次 /api/sensor，用来一边推摇杆一边看距离，
    // 直接量出「走一米要几秒」「离墙多远该停」。
    void SetSensorCallbacks(std::function<SensorSnapshot()> get_cb,
                            std::function<bool(int*)> calibrate_cb) {
        get_sensor_callback_ = std::move(get_cb);
        calibrate_sensor_callback_ = std::move(calibrate_cb);
    }

    // Debug handler 注册（/api/debug/motor_test）
    static esp_err_t debug_motor_test_handler(httpd_req_t *req);

private:
    httpd_handle_t server_handle_;
    std::function<void(int direction, int speed)> motor_control_callback_;
    std::function<void(const char* emotion)> emotion_callback_;
    std::function<MotorActionConfig()> get_motor_config_callback_;
    std::function<void(const MotorActionConfig&)> set_motor_config_callback_;
    // 测距。页面一秒轮询一次 /api/sensor，用来一边推摇杆一边看距离，
    // 直接量出"走一米要几秒""离墙多远该停"。
    std::function<SensorSnapshot()> get_sensor_callback_;
    std::function<bool(int*)> calibrate_sensor_callback_;

    // 「按住说话」的最后一次 start 时间。HTTP 是无连接的，如果 stop 请求
    // 因为网络问题没送到，麦克风会一直开着。超过 30 秒自动停。
    int64_t listen_started_ms_ = 0;

    // HTTP请求处理函数
    static esp_err_t index_get_handler(httpd_req_t *req);
    static esp_err_t control_post_handler(httpd_req_t *req);
    static esp_err_t api_control_handler(httpd_req_t *req);
    static esp_err_t api_robot_handler(httpd_req_t *req);
    static esp_err_t api_sensor_handler(httpd_req_t *req);
    static esp_err_t api_sensor_calibrate_handler(httpd_req_t *req);
    static esp_err_t api_motor_action_handler(httpd_req_t *req);
    static esp_err_t config_get_handler(httpd_req_t *req);
    static esp_err_t config_post_handler(httpd_req_t *req);
    static esp_err_t api_config_get_handler(httpd_req_t *req);
    static esp_err_t api_config_post_handler(httpd_req_t *req);
    // 7b：音量与静音。/api/audio/volume 同一路径挂 GET+POST 两个 handler。
    static esp_err_t api_volume_get_handler(httpd_req_t *req);
    static esp_err_t api_volume_post_handler(httpd_req_t *req);
    static esp_err_t api_volume_mute_handler(httpd_req_t *req);

    // 功能页：机器人模式 / 测距 / 系统（BOOT、说话、重启）
    static esp_err_t robot_get_handler(httpd_req_t *req);
    static esp_err_t sensor_get_handler(httpd_req_t *req);
    static esp_err_t sensor_post_handler(httpd_req_t *req);
    static esp_err_t system_get_handler(httpd_req_t *req);
    static esp_err_t api_system_action_handler(httpd_req_t *req);

    // CORS处理
    static esp_err_t cors_handler(httpd_req_t *req);

    // 获取HTML页面内容
    static const char* get_html_page();

    // 解析控制命令
    void parse_simple_control_command(const char* data, int& direction, int& speed);
    void parse_json_control_command(const char* data, int& direction, int& speed);

    // 配置相关方法
    void parse_config_form_data(const char* data, MotorActionConfig& config);
    static const char* get_config_html_page();
    static const char* get_robot_html_page();
    static const char* get_sensor_html_page();
    static const char* get_system_html_page();
};

#endif // WEB_SERVER_H