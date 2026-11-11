#include "web_server.h"
#include <esp_log.h>
#include <cstring>
#include <cJSON.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdlib.h>

// Forward declarations for motor control functions
extern "C" {
    // 由 application.cc 实现。用 extern 而不是 include application.h，
    // 因为 application.h 已经 include 了 web_server.h，反向 include 会成环。
    const char* HandleRobotModeStart(const char* mode);
    const char* HandleRobotModeStop();
    int         HandleRobotModeQuery();
    const char* RobotModeNameOf(int mode);
    // 7b：音量与静音
    int  HandleVolumeGet();
    int  HandleVolumeSet(int volume);
    int  HandleVolumeToggleMute();
    bool HandleVolumeIsMuted();
    // 系统页：模拟 BOOT 键 / 软件重启 / 按住说话
    void HandleSystemBoot(int long_press);
    void HandleSystemRestart();
    void HandleSystemListen(int start);
    int         HandleDeviceStateQuery();
    const char* HandleDisplayModeQuery();
}

extern "C" void HandleMotorActionForApplication(int direction, int speed, int duration_ms, int priority);
extern "C" void HandleMotorActionForEmotion(const char* emotion);
extern "C" void HandleMotorActionForDance(uint8_t speed_percent);

static const char *TAG = "WebServer";

WebServer::WebServer()
    : server_handle_(nullptr) {
}

WebServer::~WebServer() {
    Stop();
}

bool WebServer::Start(int port) {
    ESP_LOGI(TAG, "Starting web server on port %d", port);

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port;
    // v206 原版这里是 10。我们陆续加了 /api/config、/api/sensor、
    // /api/sensor/calibrate、/api/robot，一共 12 个，10 个槽装不下，
    // 后面注册的会直接打 "no slots left for registering handler" 失败。
    config.max_uri_handlers = 24;
    // 增加超时设置以更好地处理频繁请求
    config.recv_wait_timeout = 5;  // 接收超时5秒
    config.send_wait_timeout = 5;  // 发送超时5秒
    config.max_resp_headers = 8;   // 减少响应头数量

    if (httpd_start(&server_handle_, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return false;
    }

    // 注册URI处理器
    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_get_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &index_uri);

    httpd_uri_t control_uri = {
        .uri       = "/control",
        .method    = HTTP_POST,
        .handler   = control_post_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &control_uri);

    httpd_uri_t api_control_uri = {
        .uri       = "/api/control",
        .method    = HTTP_POST,
        .handler   = api_control_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_control_uri);

    httpd_uri_t api_robot_uri = {
        .uri       = "/api/robot",
        .method    = HTTP_GET,
        .handler   = api_robot_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_robot_uri);

    httpd_uri_t api_volume_get_uri = {
        .uri       = "/api/audio/volume",
        .method    = HTTP_GET,
        .handler   = api_volume_get_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_volume_get_uri);

    httpd_uri_t api_volume_post_uri = {
        .uri       = "/api/audio/volume",
        .method    = HTTP_POST,
        .handler   = api_volume_post_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_volume_post_uri);

    httpd_uri_t api_volume_mute_uri = {
        .uri       = "/api/audio/mute",
        .method    = HTTP_POST,
        .handler   = api_volume_mute_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_volume_mute_uri);

    httpd_uri_t api_sensor_uri = {
        .uri       = "/api/sensor",
        .method    = HTTP_GET,
        .handler   = api_sensor_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_sensor_uri);

    httpd_uri_t api_sensor_calibrate_uri = {
        .uri       = "/api/sensor/calibrate",
        .method    = HTTP_POST,
        .handler   = api_sensor_calibrate_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_sensor_calibrate_uri);

    httpd_uri_t api_motor_action_uri = {
        .uri       = "/api/motor/action",
        .method    = HTTP_POST,
        .handler   = api_motor_action_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_motor_action_uri);

    httpd_uri_t debug_uri = {
        .uri       = "/api/debug/motor_test",
        .method    = HTTP_POST,
        .handler   = debug_motor_test_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &debug_uri);

    // 注册配置页面处理器
    httpd_uri_t config_uri = {
        .uri       = "/config",
        .method    = HTTP_GET,
        .handler   = config_get_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &config_uri);

    httpd_uri_t config_post_uri = {
        .uri       = "/config",
        .method    = HTTP_POST,
        .handler   = config_post_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &config_post_uri);

    httpd_uri_t api_config_get_uri = {
        .uri       = "/api/config",
        .method    = HTTP_GET,
        .handler   = api_config_get_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_config_get_uri);

    httpd_uri_t api_config_post_uri = {
        .uri       = "/api/config",
        .method    = HTTP_POST,
        .handler   = api_config_post_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_config_post_uri);

    // 功能页：机器人模式 / 测距 / 系统
    httpd_uri_t robot_page_uri = {
        .uri       = "/robot",
        .method    = HTTP_GET,
        .handler   = robot_get_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &robot_page_uri);

    httpd_uri_t sensor_page_uri = {
        .uri       = "/sensor",
        .method    = HTTP_GET,
        .handler   = sensor_get_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &sensor_page_uri);

    httpd_uri_t sensor_page_post_uri = {
        .uri       = "/sensor",
        .method    = HTTP_POST,
        .handler   = sensor_post_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &sensor_page_post_uri);

    httpd_uri_t system_page_uri = {
        .uri       = "/system",
        .method    = HTTP_GET,
        .handler   = system_get_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &system_page_uri);

    httpd_uri_t api_system_action_uri = {
        .uri       = "/api/system/action",
        .method    = HTTP_POST,
        .handler   = api_system_action_handler,
        .user_ctx  = this
    };
    httpd_register_uri_handler(server_handle_, &api_system_action_uri);

    ESP_LOGI(TAG, "Web server started successfully");
    return true;
}

void WebServer::Stop() {
    if (server_handle_) {
        httpd_stop(server_handle_);
        server_handle_ = nullptr;
        ESP_LOGI(TAG, "Web server stopped");
    }
}

void WebServer::SetMotorControlCallback(std::function<void(int direction, int speed)> callback) {
    motor_control_callback_ = callback;
}

void WebServer::InvokeMotorControl(int direction, int speed) {
    if (motor_control_callback_) {
        motor_control_callback_(direction, speed);
    }
}

void WebServer::SetMotorActionConfigCallback(std::function<MotorActionConfig()> get_callback,
                                           std::function<void(const MotorActionConfig&)> set_callback) {
    get_motor_config_callback_ = get_callback;
    set_motor_config_callback_ = set_callback;
}

void WebServer::SetEmotionCallback(std::function<void(const char* emotion)> callback) {
    emotion_callback_ = callback;
}

void WebServer::SetEmotion(const char* emotion) {
    if (emotion_callback_) {
        emotion_callback_(emotion);
    }
}

esp_err_t WebServer::index_get_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, server->get_html_page(), HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

esp_err_t WebServer::control_post_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    char content[100];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }

    int direction, speed;
    server->parse_simple_control_command(content, direction, speed);

    // 调用电机控制回调（通过公共封装）
    server->InvokeMotorControl(direction, speed);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

esp_err_t WebServer::api_control_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    char content[200];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }

    // 解析JSON数据
    cJSON *root = cJSON_Parse(content);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    // v206 原版这两行是
    //   int direction = cJSON_GetObjectItem(root, "direction")->valueint;
    //   int speed = cJSON_GetObjectItem(root, "speed")->valueint;
    // 没有判空：发一个 {} 或者 {x,y} 就会解引用 NULL，直接 panic 重启。
    // 这里补上判空，缺字段时按停止处理并回错误，不再把设备带进异常。
    cJSON* jdir = cJSON_GetObjectItem(root, "direction");
    cJSON* jspd = cJSON_GetObjectItem(root, "speed");
    bool ok = (jdir != nullptr) && cJSON_IsNumber(jdir) &&
              (jspd != nullptr) && cJSON_IsNumber(jspd);
    int direction = ok ? jdir->valueint : 0;
    int speed     = ok ? jspd->valueint : 0;

    cJSON_Delete(root);

    // 调用电机控制回调（通过公共封装）
    server->InvokeMotorControl(direction, speed);

    httpd_resp_set_type(req, "application/json");
    if (!ok) {
        httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"missing direction or speed\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

// Simple struct passed to the stop task
struct debug_stop_param_t {
    WebServer* server;
    int duration_ms;
};

static void debug_stop_task(void* arg) {
    debug_stop_param_t* p = static_cast<debug_stop_param_t*>(arg);
    if (!p) vTaskDelete(NULL);
    vTaskDelay(pdMS_TO_TICKS(p->duration_ms));
    if (p->server) {
        p->server->InvokeMotorControl(0, 0); // stop via public wrapper
    }
    free(p);
    vTaskDelete(NULL);
}

esp_err_t WebServer::debug_motor_test_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }

    cJSON *root = cJSON_Parse(content);
    int direction = 4;
    int speed = 80;
    int duration = 1000;
    if (root) {
        cJSON* jdir = cJSON_GetObjectItem(root, "direction");
        cJSON* jspd = cJSON_GetObjectItem(root, "speed");
        cJSON* jdur = cJSON_GetObjectItem(root, "duration");
        if (cJSON_IsNumber(jdir)) direction = jdir->valueint;
        if (cJSON_IsNumber(jspd)) speed = jspd->valueint;
        if (cJSON_IsNumber(jdur)) duration = jdur->valueint;
        cJSON_Delete(root);
    }

    if (server) {
        server->InvokeMotorControl(direction, speed);
        // spawn a short-lived task to stop after duration ms
        debug_stop_param_t* p = (debug_stop_param_t*)malloc(sizeof(debug_stop_param_t));
        if (p) {
            p->server = server;
            p->duration_ms = duration;
            BaseType_t rc = xTaskCreate(debug_stop_task, "dbg_stop", 2048, p, tskIDLE_PRIORITY + 1, NULL);
            if (rc != pdPASS) {
                free(p);
            }
        }
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// 测距。返回 JSON，字段与 SensorSnapshot 一一对应。
// Web 页面按 age_ms 判断数据新不新鲜：超过 1000ms 就标红显示"无数据"，
// 避免把几秒前的读数当成现在的距离。
esp_err_t WebServer::api_sensor_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    SensorSnapshot snap = {};
    snap.status = 4;
    if (server != nullptr && server->get_sensor_callback_) {
        snap = server->get_sensor_callback_();
    }

    static const char* kStatusName[] = {"ok", "out_of_range", "no_target", "i2c_error", "not_ready"};
    const char* status_str = (snap.status >= 0 && snap.status <= 4) ? kStatusName[snap.status] : "unknown";

    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"distance_mm\":%d,\"status\":\"%s\",\"age_ms\":%d,"
             "\"baseline_mm\":%d,\"blocked\":%s,\"sensor_ok\":%s,\"fail_count\":%d,"
             "\"obstacle\":%d,\"edge\":%d,\"edge_confirm\":%d}",
             snap.distance_mm, status_str, snap.age_ms(),
             snap.baseline_mm, snap.blocked ? "true" : "false",
             snap.sensor_ok ? "true" : "false", snap.fail_count,
             snap.obstacle, snap.edge, snap.edge_confirm);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// 触发一次校准。阻塞约 1 秒，期间采样 20 次取中位数写 NVS。
esp_err_t WebServer::api_sensor_calibrate_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");

    if (server == nullptr || !server->calibrate_sensor_callback_) {
        httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"sensor unavailable\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    int baseline = 0;
    if (!server->calibrate_sensor_callback_(&baseline)) {
        httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"calibration failed\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    char buf[96];
    snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"baseline_mm\":%d}", baseline);
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}


// ---- 第 10 步：机器人模式 Web 接口 ----
// GET /api/robot              查当前模式
// GET /api/robot?mode=explore 启动
// GET /api/robot?mode=stop    停止
// 用 GET 是为了浏览器或 curl 一行就能测，不用发 JSON。
esp_err_t WebServer::api_volume_get_handler(httpd_req_t *req) {
    char buf[96];
    snprintf(buf, sizeof(buf), "{\"volume\":%d,\"muted\":%s}",
             HandleVolumeGet(), HandleVolumeIsMuted() ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t WebServer::api_volume_post_handler(httpd_req_t *req) {
    char content[128] = {0};
    int received = httpd_req_recv(req, content, sizeof(content) - 1);
    httpd_resp_set_type(req, "application/json");

    if (received <= 0) {
        httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"empty body\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    char buf[96];
    cJSON* root = cJSON_Parse(content);
    cJSON* jvol = (root != nullptr) ? cJSON_GetObjectItem(root, "volume") : nullptr;
    if (jvol == nullptr || !cJSON_IsNumber(jvol)) {
        if (root != nullptr) {
            cJSON_Delete(root);
        }
        httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"missing volume\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    int want = jvol->valueint;
    cJSON_Delete(root);
    int now = HandleVolumeSet(want);
    snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"volume\":%d}", now);
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t WebServer::api_volume_mute_handler(httpd_req_t *req) {
    char buf[96];
    int now = HandleVolumeToggleMute();
    snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"volume\":%d,\"muted\":%s}",
             now, HandleVolumeIsMuted() ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t WebServer::api_robot_handler(httpd_req_t *req) {
    char query[128] = {0};
    char mode[32] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "mode", mode, sizeof(mode));
    }

    char buf[256];
    httpd_resp_set_type(req, "application/json");

    if (mode[0] == '\0') {
        // /system 页面靠这条同时拿模式、设备状态、显示界面，省一个 URI 槽位
        snprintf(buf, sizeof(buf),
                 "{\"mode\":\"%s\",\"state\":%d,\"display\":\"%s\"}",
                 RobotModeNameOf(HandleRobotModeQuery()),
                 HandleDeviceStateQuery(),
                 HandleDisplayModeQuery());
        httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    if (strcmp(mode, "stop") == 0) {
        snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"message\":\"%s\"}", HandleRobotModeStop());
        httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    const char* err = HandleRobotModeStart(mode);
    if (err != nullptr && err[0] != '\0') {
        snprintf(buf, sizeof(buf), "{\"status\":\"error\",\"message\":\"%s\"}", err);
    } else {
        snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"mode\":\"%s\"}", mode);
    }
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t WebServer::api_motor_action_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }

    cJSON *root = cJSON_Parse(content);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON* action_json = cJSON_GetObjectItem(root, "action");
    if (!cJSON_IsString(action_json)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid action parameter");
        return ESP_FAIL;
    }

    std::string action = action_json->valuestring;

    // Get motor configuration for durations
    MotorActionConfig config;
    if (server->get_motor_config_callback_) {
        config = server->get_motor_config_callback_();
    }

    // Call motor action functions directly (simulating MCP tool calls) with configured durations
    try {
        ESP_LOGI(TAG, "网页动作请求: %s", action.c_str());

        if (action == "move_forward") {
            ESP_LOGI(TAG, "执行前进动作 - 速度:%d%%, 持续时间:%d ms", config.default_speed_percent, config.forward_duration_ms);
            HandleMotorActionForApplication(4, config.default_speed_percent, config.forward_duration_ms, 1);
        } else if (action == "move_backward") {
            ESP_LOGI(TAG, "执行后退动作 - 速度:%d%%, 持续时间:%d ms", config.default_speed_percent, config.backward_duration_ms);
            HandleMotorActionForApplication(2, config.default_speed_percent, config.backward_duration_ms, 1);
        } else if (action == "spin_around") {
            ESP_LOGI(TAG, "执行转圈动作 - 速度:%d%%, 持续时间:%d ms", config.default_speed_percent, config.spin_duration_ms);
            HandleMotorActionForApplication(3, config.default_speed_percent, config.spin_duration_ms, 1);
        } else if (action == "turn_left") {
            ESP_LOGI(TAG, "执行左转动作 - 速度:%d%%, 持续时间:%d ms", config.default_speed_percent, config.left_turn_duration_ms);
            HandleMotorActionForApplication(3, config.default_speed_percent, config.left_turn_duration_ms, 1);
        } else if (action == "turn_right") {
            ESP_LOGI(TAG, "执行右转动作 - 速度:%d%%, 持续时间:%d ms", config.default_speed_percent, config.right_turn_duration_ms);
            HandleMotorActionForApplication(1, config.default_speed_percent, config.right_turn_duration_ms, 1);
        } else if (action == "quick_forward") {
            ESP_LOGI(TAG, "执行快速前进动作 - 速度:%d%%, 持续时间:%d ms", config.default_speed_percent, config.quick_forward_duration_ms);
            HandleMotorActionForApplication(4, config.default_speed_percent, config.quick_forward_duration_ms, 1);
        } else if (action == "quick_backward") {
            ESP_LOGI(TAG, "执行快速后退动作 - 速度:%d%%, 持续时间:%d ms", config.default_speed_percent, config.quick_backward_duration_ms);
            HandleMotorActionForApplication(2, config.default_speed_percent, config.quick_backward_duration_ms, 1);
        } else if (action == "wiggle") {
            ESP_LOGI(TAG, "执行摆动动作 (情感:困惑)");
            server->SetEmotion("confused");
            HandleMotorActionForEmotion("confused");
        } else if (action == "dance") {
            ESP_LOGI(TAG, "执行跳舞动作 - 速度:%d%%", config.default_speed_percent);
            server->SetEmotion("excited");
            HandleMotorActionForDance(config.default_speed_percent);
        } else if (action == "stop") {
            ESP_LOGI(TAG, "执行停止动作");
            HandleMotorActionForApplication(0, 0, 0, 2); // stop, high priority
        } else if (action == "wake_up") {
            ESP_LOGI(TAG, "执行唤醒情感动作");
            server->SetEmotion("sleepy");  // 原版特殊动画：困倦效果
            HandleMotorActionForEmotion("wake");
        } else if (action == "happy") {
            ESP_LOGI(TAG, "执行开心情感动作");
            server->SetEmotion("laughing");  // 原版特殊动画：大笑动画
            HandleMotorActionForEmotion("happy");
        } else if (action == "sad") {
            ESP_LOGI(TAG, "执行悲伤情感动作");
            server->SetEmotion("crying");  // 原版特殊动画：哭泣动画
            HandleMotorActionForEmotion("sad");
        } else if (action == "thinking") {
            ESP_LOGI(TAG, "执行思考情感动作");
            server->SetEmotion("thinking");  // 原版特殊动画：思考动画
            HandleMotorActionForEmotion("thinking");
        } else if (action == "listening") {
            ESP_LOGI(TAG, "执行倾听情感动作");
            server->SetEmotion("wink");  // 原版特殊动画：眨眼动画
            HandleMotorActionForEmotion("listening");
        } else if (action == "speaking") {
            ESP_LOGI(TAG, "执行说话情感动作");
            server->SetEmotion("funny");  // 原版特殊动画：搞笑动画
            HandleMotorActionForEmotion("speaking");
        } else if (action == "excited") {
            ESP_LOGI(TAG, "执行兴奋情感动作");
            server->SetEmotion("shocked");  // 原版特殊动画：震惊动画
            HandleMotorActionForEmotion("excited");
        } else if (action == "loving") {
            ESP_LOGI(TAG, "执行爱慕情感动作");
            server->SetEmotion("kissy");  // 原版特殊动画：亲吻动画
            HandleMotorActionForEmotion("loving");
        } else if (action == "angry") {
            ESP_LOGI(TAG, "执行生气情感动作");
            server->SetEmotion("angry");  // 基础表情（生气）
            HandleMotorActionForEmotion("angry");
        } else if (action == "surprised") {
            ESP_LOGI(TAG, "执行惊讶情感动作");
            server->SetEmotion("surprised");  // 基础表情（惊讶）
            HandleMotorActionForEmotion("surprised");
        } else if (action == "confused") {
            ESP_LOGI(TAG, "执行困惑情感动作");
            server->SetEmotion("confused");  // 原版特殊动画：困惑动画
            HandleMotorActionForEmotion("confused");
        } else {
            cJSON_Delete(root);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown action");
            return ESP_FAIL;
        }
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Failed to execute motor action %s: %s", action.c_str(), e.what());
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to execute action");
        return ESP_FAIL;
    }

    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void WebServer::parse_simple_control_command(const char* data, int& direction, int& speed) {
    // 简单解析格式：direction=X,speed=Y
    char* str = strdup(data);
    char* token = strtok(str, ",");

    direction = 0;
    speed = 0;

    while (token != NULL) {
        if (strstr(token, "direction=")) {
            direction = atoi(token + 10);
        } else if (strstr(token, "speed=")) {
            speed = atoi(token + 6);
        }
        token = strtok(NULL, ",");
    }

    free(str);
}

const char* WebServer::get_html_page() {
    static const char html_page[] = R"html(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>小智小车遥控器</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            margin: 0;
            padding: 20px;
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
        }

        .container {
            background: rgba(255, 255, 255, 0.95);
            border-radius: 20px;
            padding: 30px;
            box-shadow: 0 20px 40px rgba(0,0,0,0.1);
            text-align: center;
            max-width: 400px;
            width: 100%;
        }

        h1 {
            color: #333;
            margin-bottom: 10px;
            font-size: 2.2em;
        }

        .subtitle {
            color: #666;
            margin-bottom: 30px;
            font-size: 1.1em;
        }

        .joystick-container {
            position: relative;
            width: 400px;
            height: 400px;
            margin: 0 auto 30px;
            border-radius: 50%;
            background: #f0f0f0;
            border: 3px solid #ddd;
            touch-action: none;
        }

        .joystick {
            position: absolute;
            width: 112px;
            height: 112px;
            background: linear-gradient(135deg, #4CAF50, #45a049);
            border-radius: 50%;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            box-shadow: 0 4px 8px rgba(0,0,0,0.2);
            transition: all 0.1s ease;
            cursor: pointer;
        }

        .joystick.active {
            background: linear-gradient(135deg, #2196F3, #1976D2);
            transform: translate(-50%, -50%) scale(0.95);
        }

        .direction-indicator {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            font-size: 18px;
            font-weight: bold;
            color: #333;
            pointer-events: none;
            transition: opacity 0.3s ease;
        }

        .direction-indicator.active {
            opacity: 1;
        }

        .status {
            margin-top: 20px;
            padding: 10px;
            border-radius: 10px;
            background: #f8f9fa;
            border: 1px solid #e9ecef;
        }

        .status.connected {
            background: #d4edda;
            border-color: #c3e6cb;
            color: #155724;
        }

        .status.disconnected {
            background: #f8d7da;
            border-color: #f5c6cb;
            color: #721c24;
        }

        .controls {
            margin-top: 20px;
        }

        .control-btn {
            background: linear-gradient(135deg, #FF6B6B, #EE5A24);
            color: white;
            border: none;
            padding: 12px 24px;
            border-radius: 25px;
            font-size: 16px;
            cursor: pointer;
            margin: 5px;
            transition: all 0.3s ease;
            box-shadow: 0 4px 15px rgba(255, 107, 107, 0.3);
        }

        .control-btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(255, 107, 107, 0.4);
        }

        .control-btn:active {
            transform: translateY(0);
        }

        .stop-btn {
            background: linear-gradient(135deg, #DC3545, #C82333);
        }

        .stop-btn:hover {
            box-shadow: 0 6px 20px rgba(220, 53, 69, 0.4);
        }

        /* 动作控制区域样式 */
        .actions-section {
            margin-top: 30px;
            padding: 20px;
            background: rgba(255, 255, 255, 0.9);
            border-radius: 15px;
            border: 1px solid #e9ecef;
        }

        .action-buttons {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
            gap: 20px;
        }

        .action-group {
            background: #f8f9fa;
            padding: 15px;
            border-radius: 10px;
            border: 1px solid #dee2e6;
        }

        .action-group h4 {
            margin: 0 0 15px 0;
            color: #495057;
            font-size: 1.1em;
            text-align: center;
            border-bottom: 2px solid #e9ecef;
            padding-bottom: 8px;
        }

        .action-btn {
            background: linear-gradient(135deg, #28a745, #20c997);
            color: white;
            border: none;
            padding: 10px 15px;
            border-radius: 8px;
            font-size: 14px;
            cursor: pointer;
            margin: 5px;
            transition: all 0.3s ease;
            box-shadow: 0 2px 8px rgba(40, 167, 69, 0.2);
            min-width: 100px;
        }

        .action-btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 12px rgba(40, 167, 69, 0.3);
        }

        .action-btn:active {
            transform: translateY(0);
        }

        @media (max-width: 480px) {
            .container {
                padding: 20px;
                margin: 10px;
            }

            .joystick-container {
                width: 320px;
                height: 320px;
            }

            .joystick {
                width: 96px;
                height: 96px;
            }

            h1 {
                font-size: 1.8em;
            }

            .action-buttons {
                grid-template-columns: 1fr;
                gap: 15px;
            }

            .action-btn {
                font-size: 13px;
                padding: 8px 12px;
                min-width: 80px;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚗 小智小车遥控器</h1>
        <div class="subtitle">触摸或拖拽摇杆控制小车移动</div>

        <div class="joystick-container" id="joystick-container">
            <div class="joystick" id="joystick"></div>
            <div class="direction-indicator" id="direction-indicator">⏹️</div>
        </div>

        <div class="status connected" id="status">
            <strong>状态:</strong> <span id="status-text">已连接</span>
        </div>

        <!-- 测距面板。每 500ms 拉一次 /api/sensor。
             超过 1 秒没有新数据就标红，避免把旧读数当真。 -->
        <div class="status" id="sensor-panel" style="background:#e3f2fd;color:#1565c0;">
            <strong>距离:</strong> <span id="sensor-distance">--</span>
            <span id="sensor-detail" style="font-size:13px;opacity:0.75;margin-left:8px;"></span>
        </div>

        <!-- 音量。底层是 AudioCodec 自己存 NVS 的音量（默认 70）。
             静音不是把音量设 0——固件会把 <=0 强制改成 10——
             而是记住原值再设 10，按钮变成灰色表示当前静音。 -->
        <div class="status" id="volume-panel" style="background:#f3e5f5;color:#6a1b9a;">
            <strong>音量:</strong>
            <input type="range" id="volume-slider" min="10" max="100" step="5" value="70"
                   style="width:140px;vertical-align:middle;margin:0 8px;">
            <span id="volume-value" style="font-weight:bold;">70</span>
            <button id="mute-btn" onclick="toggleMute()"
                    style="margin-left:10px;padding:2px 10px;border-radius:6px;border:1px solid #6a1b9a;background:#fff;color:#6a1b9a;cursor:pointer;">🔊</button>
        </div>

        <div class="controls">
            <button class="control-btn stop-btn" onclick="stopCar()">🛑 停止</button>
        </div>

        <!-- 功能页入口。每个功能单独一页，避免遥控页越滚越长 -->
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:14px 0;">
            <a href="/robot" class="control-btn" style="background:linear-gradient(135deg,#43a047,#2e7d32);text-decoration:none;">🤖 机器人</a>
            <a href="/sensor" class="control-btn" style="background:linear-gradient(135deg,#1e88e5,#1565c0);text-decoration:none;">📏 测距</a>
            <a href="/system" class="control-btn" style="background:linear-gradient(135deg,#8e24aa,#6a1b9a);text-decoration:none;">🔘 系统</a>
            <a href="/config" class="control-btn" style="background:linear-gradient(135deg,#FF9800,#F57C00);text-decoration:none;">⚙️ 电机</a>
        </div>

        <div class="actions-section">
            <h3 style="color: #333; margin: 20px 0 15px 0; text-align: center;">🎭 动作控制</h3>

            <div class="action-buttons">
                <!-- 基本移动动作 -->
                <div class="action-group">
                    <h4>🚗 基本移动</h4>
                    <button class="action-btn" onclick="executeAction('move_forward')">⬆️ 前进</button>
                    <button class="action-btn" onclick="executeAction('move_backward')">⬇️ 后退</button>
                    <button class="action-btn" onclick="executeAction('turn_left')">⬅️ 左转</button>
                    <button class="action-btn" onclick="executeAction('turn_right')">➡️ 右转</button>
                    <button class="action-btn" onclick="executeAction('spin_around')">🔄 转圈</button>
                </div>

                <!-- 情感动作 -->
                <div class="action-group">
                    <h4>😊 情感表达</h4>
                    <button class="action-btn" onclick="executeAction('wake_up')">🌅 唤醒</button>
                    <button class="action-btn" onclick="executeAction('happy')">😄 开心</button>
                    <button class="action-btn" onclick="executeAction('sad')">😢 悲伤</button>
                    <button class="action-btn" onclick="executeAction('thinking')">🤔 思考</button>
                    <button class="action-btn" onclick="executeAction('listening')">👂 倾听</button>
                    <button class="action-btn" onclick="executeAction('speaking')">💬 说话</button>
                    <button class="action-btn" onclick="executeAction('wiggle')">🌊 摆动</button>
                    <button class="action-btn" onclick="executeAction('dance')">💃 跳舞</button>
                </div>

                <!-- 高级情感 -->
                <div class="action-group">
                    <h4>🎭 高级情感</h4>
                    <button class="action-btn" onclick="executeAction('excited')">🤩 兴奋</button>
                    <button class="action-btn" onclick="executeAction('loving')">😍 爱慕</button>
                    <button class="action-btn" onclick="executeAction('angry')">😠 生气</button>
                    <button class="action-btn" onclick="executeAction('surprised')">😲 惊讶</button>
                    <button class="action-btn" onclick="executeAction('confused')">😕 困惑</button>
                </div>
            </div>
        </div>
    </div>

    <script>
        // === 音量 ===
        let volumeSlider = document.getElementById('volume-slider');
        let volumeValue  = document.getElementById('volume-value');
        let muteBtn      = document.getElementById('mute-btn');
        let volumeMuted  = false;

        function refreshVolume() {
            fetch('/api/audio/volume')
                .then(r => r.json())
                .then(d => {
                    volumeMuted = d.muted;
                    volumeSlider.value = d.volume;
                    volumeValue.textContent = d.volume;
                    muteBtn.textContent = volumeMuted ? '🔇' : '🔊';
                    muteBtn.style.background = volumeMuted ? '#6a1b9a' : '#fff';
                    muteBtn.style.color      = volumeMuted ? '#fff' : '#6a1b9a';
                })
                .catch(() => {});
        }

        volumeSlider.addEventListener('change', () => {
            fetch('/api/audio/volume', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({volume: parseInt(volumeSlider.value)})
            }).then(r => r.json()).then(d => {
                if (d.volume !== undefined) {
                    volumeValue.textContent = d.volume;
                    volumeSlider.value = d.volume;
                }
                refreshVolume();
            }).catch(() => {});
        });

        function toggleMute() {
            fetch('/api/audio/mute', {method: 'POST'})
                .then(r => r.json())
                .then(() => refreshVolume())
                .catch(() => {});
        }

        let joystick = document.getElementById('joystick');
        let joystickContainer = document.getElementById('joystick-container');
        let directionIndicator = document.getElementById('direction-indicator');
        let statusText = document.getElementById('status-text');

        let isDragging = false;
        let centerX = 0;
        let centerY = 0;
        let currentDirection = 0;
        let currentSpeed = 0;
        let isRequestPending = false; // 防止并发请求


        // 初始化摇杆中心位置
        function initJoystick() {
            const rect = joystickContainer.getBoundingClientRect();
            centerX = rect.left + rect.width / 2;
            centerY = rect.top + rect.height / 2;
        }

        // 更新摇杆位置
        function updateJoystickPosition(x, y) {
            const rect = joystickContainer.getBoundingClientRect();
            const containerCenterX = rect.left + rect.width / 2;
            const containerCenterY = rect.top + rect.height / 2;

            // 计算相对于容器的位置
            let relativeX = x - containerCenterX;
            let relativeY = y - containerCenterY;

            // 限制在圆形范围内
            const maxRadius = rect.width / 2 - 56;
            const distance = Math.sqrt(relativeX * relativeX + relativeY * relativeY);

            if (distance > maxRadius) {
                relativeX = (relativeX / distance) * maxRadius;
                relativeY = (relativeY / distance) * maxRadius;
            }

            // 更新摇杆位置
            joystick.style.left = `calc(50% + ${relativeX}px)`;
            joystick.style.top = `calc(50% + ${relativeY}px)`;

            // 计算方向和速度
            const normalizedX = relativeX / maxRadius;
            const normalizedY = relativeY / maxRadius;

            // 计算方向角度 (0-360度)
            let angle = Math.atan2(normalizedY, normalizedX) * (180 / Math.PI);
            if (angle < 0) angle += 360;

            // 计算速度 (0-100)
            const speed = Math.min(distance / maxRadius, 1) * 100;

            // 转换方向为整数值
            let direction = 0; // 停止
            if (speed > 5) { // 最小阈值 (降低阈值以响应点击)
                if (angle >= 315 || angle < 45) {
                    direction = 1; // 右
                } else if (angle >= 45 && angle < 135) {
                    direction = 2; // 下
                } else if (angle >= 135 && angle < 225) {
                    direction = 3; // 左
                } else if (angle >= 225 && angle < 315) {
                    direction = 4; // 上
                }
            }

            return { direction, speed: Math.round(speed) };
        }

        // 更新方向指示器
        function updateDirectionIndicator(direction, speed) {
            let icon = '⏹️';
            let text = '停止';

            if (speed > 5) {
                switch(direction) {
                    case 1: icon = '➡️'; text = '右转'; break;
                    case 2: icon = '⬇️'; text = '后退'; break;
                    case 3: icon = '⬅️'; text = '左转'; break;
                    case 4: icon = '⬆️'; text = '前进'; break;
                }
            }

            directionIndicator.textContent = icon;
            directionIndicator.classList.toggle('active', speed > 5);
        }

        // 发送控制命令
        // 发送控制命令
        async function sendControl(direction, speed) {
            // 停止命令(0, 0)优先处理，不受并发限制
            if (direction === 0 && speed === 0) {
                currentDirection = 0;
                currentSpeed = 0;
                try {
                    const response = await fetch('/api/control', {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/json',
                        },
                        body: JSON.stringify({
                            direction: 0,
                            speed: 0
                        })
                    });
                    if (!response.ok) {
                        throw new Error('Network response was not ok');
                    }
                    statusText.textContent = '已连接';
                    document.getElementById('status').className = 'status connected';
                } catch (error) {
                    console.error('Failed to send stop control:', error);
                    statusText.textContent = '连接错误';
                    document.getElementById('status').className = 'status error';
                }
                return;
            }

            if (direction === currentDirection && speed === currentSpeed) {
                return; // 避免重复发送相同命令
            }

            // 如果有请求正在进行中，跳过
            if (isRequestPending) {
                return;
            }

            currentDirection = direction;
            currentSpeed = speed;
            isRequestPending = true;

            try {
                const response = await fetch('/api/control', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        direction: direction,
                        speed: speed
                    })
                });

                if (!response.ok) {
                    throw new Error('Network response was not ok');
                }

                statusText.textContent = '已连接';
                document.getElementById('status').className = 'status connected';
            } catch (error) {
                console.error('Failed to send control:', error);
                statusText.textContent = '连接错误';
                document.getElementById('status').className = 'status error';
            } finally {
                isRequestPending = false;
            }
        }

        // 停止小车
        function stopCar() {
            // 重置状态
            isDragging = false;
            currentDirection = 0;
            currentSpeed = 0;

            // 重置UI
            joystick.style.left = '50%';
            joystick.style.top = '50%';
            joystick.classList.remove('active');
            updateDirectionIndicator(0, 0);

            // 发送停止命令
            sendControl(0, 0);
        }

        // 测距轮询。单独一个定时器，不跟摇杆抢。
        const kStatusText = ['正常', '超量程', '无目标', 'I2C 错误', '未就绪'];
        function pollSensor() {
            fetch('/api/sensor')
                .then(r => r.json())
                .then(d => {
                    const dist = document.getElementById('sensor-distance');
                    const detail = document.getElementById('sensor-detail');
                    const panel = document.getElementById('sensor-panel');

                    if (!d.sensor_ok) {
                        dist.textContent = '不可用';
                        detail.textContent = '';
                        panel.style.background = '#ffebee';
                        panel.style.color = '#c62828';
                        return;
                    }
                    if (d.age_ms > 1000) {
                        dist.textContent = '无数据';
                        detail.textContent = '上次 ' + Math.round(d.age_ms / 1000) + ' 秒前';
                        panel.style.background = '#fff8e1';
                        panel.style.color = '#ef6c00';
                        return;
                    }

                    dist.textContent = d.distance_mm + ' mm';
                    let extra = kStatusText[d.status] || ('状态 ' + d.status);
                    if (d.blocked) { extra += ' · 已拦截前进'; }
                    if (d.edge === 2) { extra += ' · 桌沿！'; }
                    if (d.baseline_mm > 0) { extra += ' · 基线 ' + d.baseline_mm + 'mm'; }
                    detail.textContent = extra;

                    // 配色优先级：桌沿 > 障碍停车 > 减速 > 正常。
                    if (d.edge === 2) {
                        panel.style.background = '#fce4ec'; panel.style.color = '#ad1457';
                    } else if (d.obstacle === 2) {
                        panel.style.background = '#ffebee'; panel.style.color = '#c62828';
                    } else if (d.obstacle === 1) {
                        panel.style.background = '#fff8e1'; panel.style.color = '#ef6c00';
                    } else {
                        panel.style.background = '#e3f2fd'; panel.style.color = '#1565c0';
                    }
                })
                .catch(() => {});
        }
        setInterval(pollSensor, 500);
        refreshVolume();
        pollSensor();


        // 执行电机动作
        async function executeAction(action) {
            try {
                const response = await fetch('/api/motor/action', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        action: action
                    })
                });

                if (!response.ok) {
                    throw new Error('Network response was not ok');
                }

                const result = await response.json();
                console.log('Action executed:', action, result);

                // 更新状态显示
                statusText.textContent = '动作执行成功';
                document.getElementById('status').className = 'status connected';

            } catch (error) {
                console.error('Failed to execute action:', action, error);
                statusText.textContent = '动作执行失败';
                document.getElementById('status').className = 'status disconnected';
            }
        }

        // 鼠标事件
        joystickContainer.addEventListener('mousedown', (e) => {
            isDragging = true;
            joystick.classList.add('active');
            initJoystick();
            const { direction, speed } = updateJoystickPosition(e.clientX, e.clientY);
            updateDirectionIndicator(direction, speed);
            sendControl(direction, speed);
        });

        document.addEventListener('mousemove', (e) => {
            if (isDragging) {
                const { direction, speed } = updateJoystickPosition(e.clientX, e.clientY);
                updateDirectionIndicator(direction, speed);
                sendControl(direction, speed);
            }
        });

        document.addEventListener('mouseup', () => {
            if (isDragging) {
                stopCar();
            }
        });

        // 触摸事件
        joystickContainer.addEventListener('touchstart', (e) => {
            e.preventDefault();
            isDragging = true;
            joystick.classList.add('active');
            initJoystick();
            const touch = e.touches[0];
            const { direction, speed } = updateJoystickPosition(touch.clientX, touch.clientY);
            updateDirectionIndicator(direction, speed);
            sendControl(direction, speed);
        });

        joystickContainer.addEventListener('touchmove', (e) => {
            e.preventDefault();
            if (isDragging) {
                const touch = e.touches[0];
                const { direction, speed } = updateJoystickPosition(touch.clientX, touch.clientY);
                updateDirectionIndicator(direction, speed);
                sendControl(direction, speed);
            }
        });

        joystickContainer.addEventListener('touchend', (e) => {
            e.preventDefault();
            if (isDragging) {
                stopCar();
            }
        });

        // 全局触摸结束事件，确保在任何地方松手都能停止
        document.addEventListener('touchend', (e) => {
            if (isDragging && e.target !== joystick && e.target !== joystickContainer) {
                stopCar();
            }
        });

        // 定期发送控制命令（当摇杆被拖拽时）
        setInterval(() => {
            if (isDragging) {
                sendControl(currentDirection, currentSpeed);
            }
        }, 200); // 每200ms发送一次，减少服务器压力

        // 初始化
        initJoystick();
        window.addEventListener('resize', initJoystick);
    </script>
</body>
</html>
)html";

    return html_page;
}


// 解析JSON控制命令
void WebServer::parse_json_control_command(const char* data, int& direction, int& speed) {
    cJSON *json = cJSON_Parse(data);
    if (json == NULL) {
        direction = 0;
        speed = 0;
        return;
    }

    cJSON *j_direction = cJSON_GetObjectItem(json, "direction");
    cJSON *j_speed = cJSON_GetObjectItem(json, "speed");

    if (cJSON_IsNumber(j_direction) && cJSON_IsNumber(j_speed)) {
        direction = j_direction->valueint;
        speed = j_speed->valueint;
    } else {
        direction = 0;
        speed = 0;
    }

    cJSON_Delete(json);
}

// 配置页面处理器
esp_err_t WebServer::config_get_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, server->get_config_html_page(), HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

// 配置页面POST处理器
esp_err_t WebServer::config_post_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    char content[512];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }

    // 解析表单数据
    MotorActionConfig config;
    server->parse_config_form_data(content, config);

    // 保存配置
    if (server->set_motor_config_callback_) {
        server->set_motor_config_callback_(config);
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, R"html(<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>配置已保存</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            margin: 0;
            padding: 20px;
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
        }

        .container {
            background: rgba(255, 255, 255, 0.95);
            border-radius: 20px;
            padding: 40px;
            box-shadow: 0 20px 40px rgba(0,0,0,0.1);
            text-align: center;
            max-width: 500px;
            width: 100%;
        }

        .success-icon {
            font-size: 4em;
            margin-bottom: 20px;
        }

        h1 {
            color: #333;
            margin-bottom: 10px;
            font-size: 2.2em;
        }

        .message {
            color: #666;
            margin-bottom: 30px;
            font-size: 1.1em;
        }

        .buttons {
            display: flex;
            gap: 15px;
            justify-content: center;
            flex-wrap: wrap;
        }

        .btn {
            background: linear-gradient(135deg, #4CAF50, #45a049);
            color: white;
            border: none;
            padding: 12px 24px;
            border-radius: 25px;
            font-size: 16px;
            cursor: pointer;
            text-decoration: none;
            display: inline-block;
            transition: all 0.3s ease;
            box-shadow: 0 4px 15px rgba(76, 175, 80, 0.3);
        }

        .btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(76, 175, 80, 0.4);
        }

        .btn.secondary {
            background: linear-gradient(135deg, #2196F3, #1976D2);
        }

        .btn.secondary:hover {
            box-shadow: 0 6px 20px rgba(33, 150, 243, 0.4);
        }

        @media (max-width: 480px) {
            .container {
                padding: 30px 20px;
            }

            .buttons {
                flex-direction: column;
                align-items: center;
            }

            .btn {
                width: 200px;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="success-icon">✅</div>
        <h1>配置已保存！</h1>
        <div class="message">您的电机动作配置已成功保存到设备中。</div>

        <div class="buttons">
            <a href="/config" class="btn secondary">⚙️ 返回配置页面</a>
            <a href="/" class="btn">🏠 返回遥控器</a>
        </div>
    </div>
</body>
</html>)html", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Configuration callback not set");
    }

    return ESP_OK;
}

// API配置GET处理器
esp_err_t WebServer::api_config_get_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    if (server->get_motor_config_callback_) {
        MotorActionConfig config = server->get_motor_config_callback_();

        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "forward_ms", config.forward_duration_ms);
        cJSON_AddNumberToObject(root, "backward_ms", config.backward_duration_ms);
        cJSON_AddNumberToObject(root, "left_turn_ms", config.left_turn_duration_ms);
        cJSON_AddNumberToObject(root, "right_turn_ms", config.right_turn_duration_ms);
        cJSON_AddNumberToObject(root, "spin_ms", config.spin_duration_ms);
        cJSON_AddNumberToObject(root, "quick_fwd_ms", config.quick_forward_duration_ms);
        cJSON_AddNumberToObject(root, "quick_bwd_ms", config.quick_backward_duration_ms);
        cJSON_AddNumberToObject(root, "def_speed_pct", config.default_speed_percent);
        cJSON_AddNumberToObject(root, "follow_far_mm",  config.follow_far_mm);
        cJSON_AddNumberToObject(root, "follow_near_mm", config.follow_near_mm);
        cJSON_AddNumberToObject(root, "follow_speed",   config.follow_speed);
        cJSON_AddNumberToObject(root, "patrol_speed",   config.patrol_speed);
        cJSON_AddNumberToObject(root, "th_blocked_mm",      config.th_blocked_mm);
        cJSON_AddNumberToObject(root, "th_blocked_exit_mm", config.th_blocked_exit_mm);
        cJSON_AddNumberToObject(root, "th_near_mm",         config.th_near_mm);
        cJSON_AddNumberToObject(root, "th_near_exit_mm",    config.th_near_exit_mm);

        char *json_str = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);

        if (json_str) {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
            cJSON_free(json_str);
        } else {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to generate JSON");
        }
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Configuration callback not set");
    }

    return ESP_OK;
}

// API配置POST处理器
esp_err_t WebServer::api_config_post_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;

    // 设置CORS头
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    char content[512];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No content");
        return ESP_FAIL;
    }

    // 解析JSON数据
    cJSON *root = cJSON_Parse(content);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    // v206 原版这 8 行全部直接解引用，缺任何字段都会 panic。
    // 改成先用现值兜底，再逐项覆盖：页面只提交改动的字段也不会把别的字段清零。
    MotorActionConfig config;
    if (server->get_motor_config_callback_) {
        config = server->get_motor_config_callback_();
    }
    auto take_int = [root](const char* key, int fallback) -> int {
        cJSON* item = cJSON_GetObjectItem(root, key);
        return (item != nullptr && cJSON_IsNumber(item)) ? item->valueint : fallback;
    };
    config.forward_duration_ms         = take_int("forward_ms",   config.forward_duration_ms);
    config.backward_duration_ms        = take_int("backward_ms",  config.backward_duration_ms);
    config.left_turn_duration_ms       = take_int("left_turn_ms", config.left_turn_duration_ms);
    config.right_turn_duration_ms      = take_int("right_turn_ms",config.right_turn_duration_ms);
    config.spin_duration_ms            = take_int("spin_ms",      config.spin_duration_ms);
    config.quick_forward_duration_ms   = take_int("quick_fwd_ms", config.quick_forward_duration_ms);
    config.quick_backward_duration_ms  = take_int("quick_bwd_ms", config.quick_backward_duration_ms);
    config.default_speed_percent       = take_int("def_speed_pct",config.default_speed_percent);
    config.follow_far_mm               = take_int("follow_far_mm",  config.follow_far_mm);
    config.follow_near_mm              = take_int("follow_near_mm", config.follow_near_mm);
    config.follow_speed                = take_int("follow_speed",   config.follow_speed);
    config.patrol_speed                = take_int("patrol_speed",   config.patrol_speed);
    config.th_blocked_mm               = take_int("th_blocked_mm",      config.th_blocked_mm);
    config.th_blocked_exit_mm          = take_int("th_blocked_exit_mm", config.th_blocked_exit_mm);
    config.th_near_mm                  = take_int("th_near_mm",         config.th_near_mm);
    config.th_near_exit_mm             = take_int("th_near_exit_mm",    config.th_near_exit_mm);

    cJSON_Delete(root);

    // 保存配置
    if (server->set_motor_config_callback_) {
        server->set_motor_config_callback_(config);

        // 在串口中输出保存的配置信息
        ESP_LOGI(TAG, "网页配置已保存:");
        ESP_LOGI(TAG, "  前进时间: %d ms", config.forward_duration_ms);
        ESP_LOGI(TAG, "  后退时间: %d ms", config.backward_duration_ms);
        ESP_LOGI(TAG, "  左转时间: %d ms", config.left_turn_duration_ms);
        ESP_LOGI(TAG, "  右转时间: %d ms", config.right_turn_duration_ms);
        ESP_LOGI(TAG, "  转圈时间: %d ms", config.spin_duration_ms);
        ESP_LOGI(TAG, "  快速前进时间: %d ms", config.quick_forward_duration_ms);
        ESP_LOGI(TAG, "  快速后退时间: %d ms", config.quick_backward_duration_ms);
        ESP_LOGI(TAG, "  默认速度: %d%%", config.default_speed_percent);

        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"status\":\"success\"}", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Configuration callback not set");
    }

    return ESP_OK;
}

// 解析配置表单数据
void WebServer::parse_config_form_data(const char* data, MotorActionConfig& config) {
    char* str = strdup(data);
    char* token = strtok(str, "&");

    while (token != NULL) {
        char* eq_pos = strchr(token, '=');
        if (eq_pos) {
            *eq_pos = '\0';
            char* key = token;
            char* value = eq_pos + 1;

            if (strcmp(key, "follow_far_mm") == 0) {
                config.follow_far_mm = atoi(value);
            } else if (strcmp(key, "follow_near_mm") == 0) {
                config.follow_near_mm = atoi(value);
            } else if (strcmp(key, "follow_speed") == 0) {
                config.follow_speed = atoi(value);
            } else if (strcmp(key, "patrol_speed") == 0) {
                config.patrol_speed = atoi(value);
            } else if (strcmp(key, "th_blocked_mm") == 0) {
                config.th_blocked_mm = atoi(value);
            } else if (strcmp(key, "th_blocked_exit_mm") == 0) {
                config.th_blocked_exit_mm = atoi(value);
            } else if (strcmp(key, "th_near_mm") == 0) {
                config.th_near_mm = atoi(value);
            } else if (strcmp(key, "th_near_exit_mm") == 0) {
                config.th_near_exit_mm = atoi(value);
            } else if (strcmp(key, "forward_ms") == 0) {
                config.forward_duration_ms = atoi(value);
            } else if (strcmp(key, "backward_ms") == 0) {
                config.backward_duration_ms = atoi(value);
            } else if (strcmp(key, "left_turn_ms") == 0) {
                config.left_turn_duration_ms = atoi(value);
            } else if (strcmp(key, "right_turn_ms") == 0) {
                config.right_turn_duration_ms = atoi(value);
            } else if (strcmp(key, "spin_ms") == 0) {
                config.spin_duration_ms = atoi(value);
            } else if (strcmp(key, "quick_fwd_ms") == 0) {
                config.quick_forward_duration_ms = atoi(value);
            } else if (strcmp(key, "quick_bwd_ms") == 0) {
                config.quick_backward_duration_ms = atoi(value);
            } else if (strcmp(key, "def_speed_pct") == 0) {
                config.default_speed_percent = atoi(value);
            }
        }
        token = strtok(NULL, "&");
    }

    free(str);
}

// 获取配置页面HTML
const char* WebServer::get_config_html_page() {
    static const char config_html_page[] = R"html(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>电机动作配置</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            margin: 0;
            padding: 20px;
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
        }

        .container {
            background: rgba(255, 255, 255, 0.95);
            border-radius: 20px;
            padding: 30px;
            box-shadow: 0 20px 40px rgba(0,0,0,0.1);
            max-width: 600px;
            width: 100%;
        }

        h1 {
            color: #333;
            text-align: center;
            margin-bottom: 30px;
        }

        .form-group {
            margin-bottom: 20px;
        }

        label {
            display: block;
            margin-bottom: 5px;
            font-weight: bold;
            color: #555;
        }

        input[type="number"] {
            width: 100%;
            padding: 10px;
            border: 2px solid #ddd;
            border-radius: 8px;
            font-size: 16px;
            transition: border-color 0.3s ease;
        }

        input[type="number"]:focus {
            outline: none;
            border-color: #4CAF50;
        }

        .unit {
            color: #666;
            font-size: 14px;
            margin-left: 5px;
        }

        .description {
            color: #777;
            font-size: 14px;
            margin-top: 3px;
            font-weight: normal;
        }

        .buttons {
            text-align: center;
            margin-top: 30px;
        }

        .btn {
            background: linear-gradient(135deg, #4CAF50, #45a049);
            color: white;
            border: none;
            padding: 12px 30px;
            border-radius: 25px;
            font-size: 16px;
            cursor: pointer;
            margin: 0 10px;
            text-decoration: none;
            display: inline-block;
            transition: all 0.3s ease;
        }

        .btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(76, 175, 80, 0.4);
        }

        .btn.secondary {
            background: linear-gradient(135deg, #2196F3, #1976D2);
        }

        .btn.secondary:hover {
            box-shadow: 0 6px 20px rgba(33, 150, 243, 0.4);
        }

        .grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
        }

        @media (max-width: 480px) {
            .grid {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>⚙️ 电机动作配置</h1>
        <form method="POST" action="/config">
            <div class="grid">
                <div class="form-group">
                    <label for="forward_ms">前进时间</label>
                    <input type="number" id="forward_ms" name="forward_ms" min="100" max="30000" step="100" required>
                    <span class="unit">毫秒</span>
                    <div class="description">默认前进动作的持续时间</div>
                </div>

                <div class="form-group">
                    <label for="backward_ms">后退时间</label>
                    <input type="number" id="backward_ms" name="backward_ms" min="100" max="30000" step="100" required>
                    <span class="unit">毫秒</span>
                    <div class="description">默认后退动作的持续时间</div>
                </div>

                <div class="form-group">
                    <label for="left_turn_ms">左转时间</label>
                    <input type="number" id="left_turn_ms" name="left_turn_ms" min="100" max="10000" step="50" required>
                    <span class="unit">毫秒</span>
                    <div class="description">左转动作的持续时间</div>
                </div>

                <div class="form-group">
                    <label for="right_turn_ms">右转时间</label>
                    <input type="number" id="right_turn_ms" name="right_turn_ms" min="100" max="10000" step="50" required>
                    <span class="unit">毫秒</span>
                    <div class="description">右转动作的持续时间</div>
                </div>

                <div class="form-group">
                    <label for="spin_ms">转圈时间</label>
                    <input type="number" id="spin_ms" name="spin_ms" min="500" max="10000" step="100" required>
                    <span class="unit">毫秒</span>
                    <div class="description">转圈动作的持续时间</div>
                </div>


                <div class="form-group">
                    <label for="def_speed_pct">默认速度</label>
                    <input type="number" id="def_speed_pct" name="def_speed_pct" min="10" max="100" step="5" required>
                    <span class="unit">%</span>
                    <div class="description">语音动作的默认速度</div>
                </div>

                <div class="form-group">
                    <label for="follow_far_mm">跟随：追进距离</label>
                    <input type="number" id="follow_far_mm" name="follow_far_mm" min="50" max="400" step="10" required>
                    <span class="unit">毫米</span>
                    <div class="description">超过这个距离才开始往前追。电池电压低时电机会带不动，速度调高之后这个值也要跟着挪</div>
                </div>

                <div class="form-group">
                    <label for="follow_near_mm">跟随：后退距离</label>
                    <input type="number" id="follow_near_mm" name="follow_near_mm" min="30" max="300" step="10" required>
                    <span class="unit">毫米</span>
                    <div class="description">近于这个距离就往后撤。必须比上面的追进距离小</div>
                </div>

                <div class="form-group">
                    <label for="follow_speed">跟随：速度</label>
                    <input type="number" id="follow_speed" name="follow_speed" min="30" max="100" step="5" required>
                    <span class="unit">%</span>
                    <div class="description">追进和后退都用这个速度。电池电压掉下来时 55% 会带不动，往上提到 70 左右</div>
                </div>

                <div class="form-group">
                    <label for="patrol_speed">巡检：速度</label>
                    <input type="number" id="patrol_speed" name="patrol_speed" min="30" max="100" step="5" required>
                    <span class="unit">%</span>
                    <div class="description">直行、后退、转向都用这个速度。40% 带不动电机，所以默认给 55</div>
                </div>

                <!-- 测距的四个阈值已搬到 /sensor 单独页面。 -->
            </div>

            <div class="buttons">
                <button type="submit" class="btn">💾 保存配置</button>
                <a href="/" class="btn secondary">🏠 返回遥控器</a>
            </div>
        </form>
    </div>

    <script>
        // 页面加载时获取当前配置
        window.onload = function() {
            fetch('/api/config')
                .then(response => response.json())
                .then(config => {
                    document.getElementById('forward_ms').value = config.forward_ms;
                    document.getElementById('backward_ms').value = config.backward_ms;
                    document.getElementById('left_turn_ms').value = config.left_turn_ms;
                    document.getElementById('right_turn_ms').value = config.right_turn_ms;
                    document.getElementById('spin_ms').value = config.spin_ms;
                    document.getElementById('def_speed_pct').value = config.def_speed_pct;
                    document.getElementById('follow_far_mm').value = config.follow_far_mm;
                    document.getElementById('follow_near_mm').value = config.follow_near_mm;
                    document.getElementById('follow_speed').value = config.follow_speed;
                    document.getElementById('patrol_speed').value = config.patrol_speed;
                    document.getElementById('th_blocked_mm').value = config.th_blocked_mm;
                    document.getElementById('th_blocked_exit_mm').value = config.th_blocked_exit_mm;
                    document.getElementById('th_near_mm').value = config.th_near_mm;
                    document.getElementById('th_near_exit_mm').value = config.th_near_exit_mm;
                })
                .catch(error => console.error('Failed to load config:', error));
        };

        // 处理表单提交
        document.getElementById('config-form').addEventListener('submit', function(e) {
            e.preventDefault(); // 阻止默认表单提交

            const formData = new FormData(this);
            const data = Object.fromEntries(formData.entries());

            fetch('/api/config', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(data)
            })
            .then(response => response.json())
            .then(result => {
                if (result.status === 'success') {
                    // 显示成功消息
                    alert('配置保存成功！');
                    // 自动跳转回遥控器界面
                    window.location.href = '/';
                } else {
                    alert('配置保存失败，请重试');
                }
            })
            .catch(error => {
                console.error('Failed to save config:', error);
                alert('配置保存失败，请检查网络连接');
            });
        });
    </script>
</body>
</html>
)html";

    return config_html_page;
}

esp_err_t WebServer::robot_get_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, server->get_robot_html_page(), HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

const char* WebServer::get_robot_html_page() {
    static const char page[] = R"html(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>机器人模式 - 小智</title>
<style>
*{box-sizing:border-box}
body{font-family:-apple-system,'Segoe UI',sans-serif;background:linear-gradient(135deg,#667eea,#764ba2);margin:0;padding:20px;min-height:100vh}
.box{background:rgba(255,255,255,.96);border-radius:16px;padding:24px;max-width:520px;margin:0 auto;box-shadow:0 10px 30px rgba(0,0,0,.15)}
h1{margin:0 0 6px;font-size:1.5em;color:#333}
.sub{color:#777;font-size:.9em;margin-bottom:18px}
.row{display:flex;justify-content:space-between;padding:9px 0;border-bottom:1px solid #eee;font-size:.95em}
.row:last-child{border:none}
.k{color:#666}.v{font-weight:600;color:#333}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin:16px 0}
button{padding:14px 8px;border:none;border-radius:10px;background:linear-gradient(135deg,#667eea,#764ba2);color:#fff;font-size:.95em;cursor:pointer;line-height:1.35}
button:active{transform:scale(.97)}
button.gray{background:#e0e0e0;color:#555}
button:disabled{opacity:.4;cursor:not-allowed}
button small{display:block;font-size:.72em;opacity:.85;margin-top:3px}
a{display:block;text-align:center;margin-top:18px;color:#667eea;text-decoration:none}
</style>
</head>
<body>
<div class="box">
<h1>机器人模式</h1>
<div class="sub">模式运行期间小智不会自动休眠</div>
<div class="row"><span class="k">当前模式</span><span class="v" id="mode">--</span></div>
<div class="row"><span class="k">距离传感器</span><span class="v" id="sensor">--</span></div>
<div class="row"><span class="k">设备状态</span><span class="v" id="state">--</span></div>
<div class="grid">
  <button onclick="setMode('explore')">探索<small>随机走动 + 音效</small></button>
  <button onclick="setMode('free')">自由<small>只动表情</small></button>
  <button onclick="setMode('follow')" id="btn-follow">跟随<small>保持距离</small></button>
  <button onclick="setMode('edge_patrol')" id="btn-patrol">边缘巡检<small>桌沿后退转向</small></button>
</div>
<button class="gray" style="width:100%" onclick="setMode('stop')">停止当前模式</button>
<a href="/">← 返回遥控器</a>
</div>
<script>
const NAME={none:'无',explore:'探索',free:'自由',follow:'跟随',edge_patrol:'边缘巡检'};
function setMode(m){
  fetch('/api/robot?mode='+encodeURIComponent(m)).then(r=>r.json()).then(d=>{
    if(d.status==='error'){alert(d.message||'失败');}
    refresh();
  }).catch(()=>alert('请求失败，设备可能断线'));
}
function refresh(){
  fetch('/api/robot').then(r=>r.json()).then(d=>{
    document.getElementById('mode').textContent=NAME[d.mode]||d.mode;
  }).catch(()=>{document.getElementById('mode').textContent='离线';});
  fetch('/api/sensor').then(r=>r.json()).then(d=>{
    var el=document.getElementById('sensor');
    if(!d.sensor_ok){el.textContent='不可用';el.style.color='#c62828';}
    else if(d.age_ms>1000){el.textContent='无数据';el.style.color='#c62828';}
    else{el.textContent=d.distance_mm+' mm';el.style.color='#333';}
    var ok=d.sensor_ok && d.age_ms<=1000;
    document.getElementById('btn-follow').disabled=!ok;
    document.getElementById('btn-patrol').disabled=!ok;
  }).catch(()=>{});
}
setInterval(refresh,1000);
refresh();
</script>
</body>
</html>
)html";
    return page;
}

esp_err_t WebServer::sensor_get_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, server->get_sensor_html_page(), HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

const char* WebServer::get_sensor_html_page() {
    static const char page[] = R"html(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>测距 - 小智</title>
<style>
*{box-sizing:border-box}
body{font-family:-apple-system,'Segoe UI',sans-serif;background:linear-gradient(135deg,#667eea,#764ba2);margin:0;padding:20px;min-height:100vh}
.box{background:rgba(255,255,255,.96);border-radius:16px;padding:24px;max-width:520px;margin:0 auto 16px;box-shadow:0 10px 30px rgba(0,0,0,.15)}
h1{margin:0 0 6px;font-size:1.5em;color:#333}
.sub{color:#777;font-size:.9em;margin-bottom:18px}
.row{display:flex;justify-content:space-between;padding:9px 0;border-bottom:1px solid #eee;font-size:.95em}
.k{color:#666}.v{font-weight:600;color:#333}
.field{margin:12px 0}
.field label{display:block;font-size:.9em;color:#555;margin-bottom:5px}
.field input{width:100%;padding:10px;border:1px solid #ddd;border-radius:8px;font-size:1em}
.field .desc{font-size:.78em;color:#999;margin-top:4px}
button{width:100%;padding:14px;border:none;border-radius:10px;background:linear-gradient(135deg,#667eea,#764ba2);color:#fff;font-size:.95em;cursor:pointer;margin-top:8px}
button.gray{background:#e0e0e0;color:#555}
a{display:block;text-align:center;margin-top:18px;color:#667eea;text-decoration:none}
.big{font-size:2em;font-weight:700;color:#667eea;text-align:center;padding:10px 0}
</style>
</head>
<body>
<div class="box">
<h1>测距</h1>
<div class="sub">VL6180X · 安装角度朝前下方，平地读数约 150mm</div>
<div class="big" id="dist">--</div>
<div class="row"><span class="k">状态码</span><span class="v" id="status">--</span></div>
<div class="row"><span class="k">数据新鲜度</span><span class="v" id="age">--</span></div>
<div class="row"><span class="k">障碍</span><span class="v" id="obs">--</span></div>
<div class="row"><span class="k">桌沿</span><span class="v" id="edge">--</span></div>
<div class="row"><span class="k">基线（校准值）</span><span class="v" id="base">--</span></div>
</div>
<div class="box">
<h1 style="font-size:1.15em">阈值</h1>
<div class="sub">顺序必须是：停车 &lt; 停车解除 &lt; 减速 &lt; 减速解除</div>
<form id="th-form">
  <div class="field"><label>停车距离 (mm)</label><input type="number" name="th_blocked_mm" id="f1"><div class="desc">小于这个值：停车 + 响 5cm 音效</div></div>
  <div class="field"><label>停车解除 (mm)</label><input type="number" name="th_blocked_exit_mm" id="f2"><div class="desc">大于这个值：解除停车，响 10cm 音效</div></div>
  <div class="field"><label>减速距离 (mm)</label><input type="number" name="th_near_mm" id="f3"><div class="desc">进入这个区间：限速 70%</div></div>
  <div class="field"><label>减速解除 (mm)</label><input type="number" name="th_near_exit_mm" id="f4"><div class="desc">大于这个值：恢复正常速度</div></div>
  <button type="submit" id="save-btn">保存</button>
  <div id="save-msg" style="font-size:.85em;min-height:18px;margin-top:6px;text-align:center"></div>
</form>
<button class="gray" onclick="calibrate()">重新校准基线</button>
<div class="sub" style="margin-top:8px;font-size:.78em">校准会让电机先停一下，采样 20 次取中位数</div>
<a href="/">← 返回遥控器</a>
</div>
<script>
const ST={0:'正常',1:'超量程',2:'目标太弱',3:'环境光异常',4:'未就绪'};
const OBS={0:'通畅',1:'靠近（减速）',2:'停车'};
const EDGE={0:'未知',1:'在桌上',2:'桌沿'};
function refresh(){
  fetch('/api/sensor').then(r=>r.json()).then(d=>{
    document.getElementById('dist').textContent = d.sensor_ok ? d.distance_mm+' mm' : '不可用';
    document.getElementById('dist').style.color = d.sensor_ok ? (d.age_ms>1000?'#c62828':'#667eea') : '#c62828';
    document.getElementById('status').textContent = ST[d.status]||('代码 '+d.status);
    document.getElementById('age').textContent = d.age_ms+' ms'+(d.age_ms>1000?'（过期）':'');
    document.getElementById('obs').textContent = OBS[d.obstacle]||d.obstacle;
    document.getElementById('edge').textContent = EDGE[d.edge]||d.edge;
    document.getElementById('base').textContent = d.baseline_mm>0 ? d.baseline_mm+' mm' : '未校准';
  }).catch(()=>{document.getElementById('dist').textContent='离线';});
}
function loadCfg(){
  fetch('/api/config').then(r=>r.json()).then(d=>{
    document.getElementById('f1').value=d.th_blocked_mm;
    document.getElementById('f2').value=d.th_blocked_exit_mm;
    document.getElementById('f3').value=d.th_near_mm;
    document.getElementById('f4').value=d.th_near_exit_mm;
  }).catch(()=>{});
}
document.getElementById('th-form').addEventListener('submit', function(e){
  e.preventDefault();
  var body='th_blocked_mm='+encodeURIComponent(document.getElementById('f1').value)
         +'&th_blocked_exit_mm='+encodeURIComponent(document.getElementById('f2').value)
         +'&th_near_mm='+encodeURIComponent(document.getElementById('f3').value)
         +'&th_near_exit_mm='+encodeURIComponent(document.getElementById('f4').value);
  var msg=document.getElementById('save-msg');
  var btn=document.getElementById('save-btn');
  btn.disabled=true;btn.textContent='保存中…';
  fetch('/sensor',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body})
   .then(r=>r.json()).then(d=>{
     btn.disabled=false;btn.textContent='保存';
     if(d.status==='ok'){msg.style.color='#2e7d32';msg.textContent='已保存';}
     else{msg.style.color='#c62828';msg.textContent=d.message||'保存失败';}
   }).catch(()=>{
     btn.disabled=false;btn.textContent='保存';
     msg.style.color='#c62828';msg.textContent='请求失败';
   });
});

function calibrate(){
  if(!confirm('校准期间电机会停一下，继续？'))return;
  fetch('/api/sensor/calibrate',{method:'POST'}).then(r=>r.json()).then(d=>{
    if(d.status==='ok')alert('校准完成，基线 '+d.baseline_mm+' mm');
    else alert('校准失败：'+(d.message||'未知原因'));
  }).catch(()=>alert('请求失败'));
}
setInterval(refresh,500);
refresh();loadCfg();
</script>
</body>
</html>
)html";
    return page;
}

esp_err_t WebServer::sensor_post_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;
    char buf[512];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty body");
        return ESP_OK;
    }
    buf[ret] = '\0';

    MotorActionConfig config;
    if (server->get_motor_config_callback_) {
        config = server->get_motor_config_callback_();
    }
    auto take = [&buf](const char* key, int fallback) -> int {
        const char* p = strstr(buf, key);
        if (p == nullptr) return fallback;
        p += strlen(key);
        if (*p == '=') p++;
        int v = atoi(p);
        return v > 0 ? v : fallback;
    };
    config.th_blocked_mm      = take("th_blocked_mm",      config.th_blocked_mm);
    config.th_blocked_exit_mm = take("th_blocked_exit_mm", config.th_blocked_exit_mm);
    config.th_near_mm         = take("th_near_mm",         config.th_near_mm);
    config.th_near_exit_mm    = take("th_near_exit_mm",    config.th_near_exit_mm);

    // 顺序校验放在 Web 层：传感器那边虽然也会拒，但那边只打日志，
    // 用户看不到。这里直接回错误，页面能立刻提示哪里不对。
    httpd_resp_set_type(req, "application/json");
    if (!(config.th_blocked_mm < config.th_blocked_exit_mm &&
          config.th_blocked_exit_mm < config.th_near_mm &&
          config.th_near_mm < config.th_near_exit_mm)) {
        ESP_LOGW(TAG, "sensor thresholds rejected: %d/%d/%d/%d",
                 config.th_blocked_mm, config.th_blocked_exit_mm,
                 config.th_near_mm, config.th_near_exit_mm);
        httpd_resp_send(req,
            "{\"status\":\"error\",\"message\":\"顺序必须是：停车 < 停车解除 < 减速 < 减速解除\"}",
            HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    if (server->set_motor_config_callback_) {
        server->set_motor_config_callback_(config);
    }

    httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t WebServer::system_get_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, server->get_system_html_page(), HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

const char* WebServer::get_system_html_page() {
    static const char page[] = R"html(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>系统 - 小智</title>
<style>
*{box-sizing:border-box}
body{font-family:-apple-system,'Segoe UI',sans-serif;background:linear-gradient(135deg,#667eea,#764ba2);margin:0;padding:20px;min-height:100vh}
.box{background:rgba(255,255,255,.96);border-radius:16px;padding:24px;max-width:520px;margin:0 auto 16px;box-shadow:0 10px 30px rgba(0,0,0,.15)}
h1{margin:0 0 6px;font-size:1.5em;color:#333}
h2{margin:0 0 4px;font-size:1.05em;color:#444}
.sub{color:#777;font-size:.85em;margin-bottom:16px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin:14px 0}
button{padding:14px 8px;border:none;border-radius:10px;background:linear-gradient(135deg,#667eea,#764ba2);color:#fff;font-size:.95em;cursor:pointer;line-height:1.35;width:100%}
button:active{transform:scale(.97)}
button.gray{background:#e0e0e0;color:#555}
button.red{background:linear-gradient(135deg,#e53935,#c62828)}
button.talk{background:linear-gradient(135deg,#43a047,#2e7d32)}
button small{display:block;font-size:.72em;opacity:.85;margin-top:3px}
.row{display:flex;justify-content:space-between;padding:9px 0;border-bottom:1px solid #eee;font-size:.95em}
.row:last-child{border:none}
.k{color:#666}.v{font-weight:600;color:#333}
a{display:block;text-align:center;margin-top:18px;color:#667eea;text-decoration:none}
#talk-state{text-align:center;font-size:.85em;color:#888;margin-top:8px;min-height:18px}
</style>
</head>
<body>
<div class="box">
<h1>系统</h1>
<div class="sub">这里的按钮等价于设备上的物理按键</div>
<div class="row"><span class="k">设备状态</span><span class="v" id="state">--</span></div>
<div class="row"><span class="k">显示界面</span><span class="v" id="display">--</span></div>
</div>

<div class="box">
<h2>按住说话</h2>
<div class="sub">按住下面的按钮开始说，松开就发出去</div>
<button class="talk" id="talk-btn">按住说话</button>
<div id="talk-state"></div>
</div>

<div class="box">
<h2>BOOT 键</h2>
<div class="sub">跟板子上那颗 BOOT 一样</div>
<div class="grid">
  <button onclick="act('boot_short')">短按<small>开 / 关对话</small></button>
  <button onclick="act('boot_long')">长按<small>切显示界面</small></button>
</div>
</div>

<div class="box">
<h2>重启</h2>
<div class="sub">同 RESET 键。重启后本页会断，10 秒后刷新</div>
<button class="red" id="rst-btn" onclick="restart()">重启设备</button>
</div>
<a href="/">← 返回遥控器</a>

<script>
const ST={0:'未知',1:'开机中',2:'配网中',3:'待机',4:'连接中',5:'正在听',6:'说话中',7:'升级中',8:'激活中',9:'音频测试',10:'致命错误'};
const DIS={default:'官方界面',eyes:'眼睛',sleep:'大时钟',music:'音乐'};
function refresh(){
  fetch('/api/robot').then(r=>r.json()).then(d=>{
    document.getElementById('state').textContent=ST[d.state]||('代码 '+d.state);
    document.getElementById('display').textContent=DIS[d.display]||d.display||'--';
  }).catch(()=>{document.getElementById('state').textContent='离线';});
}
function act(a){
  fetch('/api/system/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:a})})
   .then(r=>r.json()).then(d=>{if(d.status!=='ok')alert(d.message||'失败');refresh();})
   .catch(()=>alert('请求失败'));
}
var talking=false;
function talkStart(e){
  e.preventDefault();
  if(talking)return;
  talking=true;
  document.getElementById('talk-btn').textContent='正在听…（松开结束）';
  document.getElementById('talk-state').textContent='麦克风已打开';
  fetch('/api/system/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'listen_start'})}).catch(()=>{});
}
function talkStop(e){
  if(!talking)return;
  talking=false;
  document.getElementById('talk-btn').textContent='按住说话';
  document.getElementById('talk-state').textContent='已发送';
  fetch('/api/system/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'listen_stop'})}).catch(()=>{});
}
var btn=document.getElementById('talk-btn');
btn.addEventListener('mousedown',talkStart);
btn.addEventListener('mouseup',talkStop);
btn.addEventListener('mouseleave',talkStop);
btn.addEventListener('touchstart',talkStart,{passive:false});
btn.addEventListener('touchend',talkStop);
btn.addEventListener('touchcancel',talkStop);

var rstArmed=false,rstTimer=null;
function restart(){
  var b=document.getElementById('rst-btn');
  if(!rstArmed){
    rstArmed=true;
    b.textContent='再点一次确认重启';
    rstTimer=setTimeout(function(){rstArmed=false;b.textContent='重启设备';},3000);
    return;
  }
  clearTimeout(rstTimer);
  b.textContent='正在重启…';
  b.disabled=true;
  fetch('/api/system/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'restart'})}).catch(()=>{});
  setTimeout(function(){location.reload();},10000);
}
setInterval(refresh,1000);
refresh();
</script>
</body>
</html>
)html";
    return page;
}

// /api/system/state 用现有 /api/sensor 之外单独一条，返回设备状态与显示界面。
// 它和 /api/system/action 共用一个 handler 不划算（方法不同），所以单独走
// action 里的 GET 分支——但 httpd 一条 URI 只能挂一个方法，这里换个思路：
// state 直接塞进 action 的 GET。为省槽位，state 走 /api/robot 的响应扩展，
// 页面改成读它。见下面 api_system_action_handler 的注释。

esp_err_t WebServer::api_system_action_handler(httpd_req_t *req) {
    WebServer* server = (WebServer*)req->user_ctx;
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty body");
        return ESP_OK;
    }
    buf[ret] = '\0';

    httpd_resp_set_type(req, "application/json");

    if (strstr(buf, "\"restart\"") != nullptr) {
        httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
        // 延迟一点点让响应先发出去，再重启
        vTaskDelay(pdMS_TO_TICKS(200));
        HandleSystemRestart();
        return ESP_OK;
    }
    if (strstr(buf, "\"boot_short\"") != nullptr) {
        HandleSystemBoot(0);
        httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    if (strstr(buf, "\"boot_long\"") != nullptr) {
        HandleSystemBoot(1);
        httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    if (strstr(buf, "\"listen_start\"") != nullptr) {
        server->listen_started_ms_ = esp_timer_get_time() / 1000;
        HandleSystemListen(1);
        httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    if (strstr(buf, "\"listen_stop\"") != nullptr) {
        server->listen_started_ms_ = 0;
        HandleSystemListen(0);
        httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    // 兜底：不管发来什么，先确保麦克风关掉，别让它一直开着
    if (server->listen_started_ms_ != 0 &&
        esp_timer_get_time() / 1000 - server->listen_started_ms_ > 30000) {
        ESP_LOGW(TAG, "listen timeout 30 s, force stop");
        server->listen_started_ms_ = 0;
        HandleSystemListen(0);
    }
    httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"unknown action\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
