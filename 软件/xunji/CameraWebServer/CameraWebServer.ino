//======================================================================
// PART 1: CameraWebServer.ino 原始代码的顶部和库引用
//======================================================================

#include "esp_camera.h"
#include <WiFi.h>

// ---------------------------------------------------------------------
// 【新增】BLE 和 FreeRTOS 库引用
// ---------------------------------------------------------------------
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "具身智能机器人-2.4G";
const char *password = "jushen123";

void startCameraServer();
void setupLedFlash();

//======================================================================
// PART 2: 【新增】BLE 变量、电机引脚、FreeRTOS 结构体和函数定义
//======================================================================

// ---------------------------------------------------------------------
// 1. BLE 服务使用 Nordic UART Profile 
// ---------------------------------------------------------------------
#define SERVICE_UUID       "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_CHARACTERISTIC_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic* pTxCharacteristic = nullptr; // 用来向 Python 发送数据的特征（通知用）
bool deviceConnected = false;          // 当前是否有手机/电脑通过蓝牙连上了


// ---------------------------------------------------------------------
// 2. 电机引脚定义（已修正为安全引脚）
// ---------------------------------------------------------------------
// 右电机引脚
#define R_STEP_PIN   1 // 右电机脉冲信号 
#define R_DIR_PIN   2 // 右电机方向信号
#define R_ENABLE_PIN  3 // 右电机使能信号

// 左电机引脚
#define L_STEP_PIN  14  // 左电机脉冲信号
#define L_DIR_PIN   15  // 左电机方向信号
#define L_ENABLE_PIN 16  // 左电机使能信号


// ---------------------------------------------------------------------
// 3. FreeRTOS 任务间通信结构体 (用于传递指令)
// ---------------------------------------------------------------------
typedef enum {
  CMD_RP1,    // 右电机 正向一步
  CMD_RP0,    // 右电机 反向一步
  CMD_LP1,    // 左电机 正向一步
  CMD_LP0,    // 左电机 反向一步
  CMD_ENABLE_ON, // 使能开启
  CMD_ENABLE_OFF // 使能关闭
} MotorCommand_t;

// 队列句柄，用于在 BLE 和 Motor Task 之间安全传递指令
QueueHandle_t motorCommandQueue; 

// ---------------------------------------------------------------------
// 4. 电机使能控制函数
// ---------------------------------------------------------------------
void enableMotors() {
 digitalWrite(R_ENABLE_PIN, LOW);  // 低电平 = 使能
 digitalWrite(L_ENABLE_PIN, LOW);
 Serial.println("电机已使能（有力矩）");
}

void disableMotors() {
 digitalWrite(R_ENABLE_PIN, HIGH); // 高电平 = 关闭使能
 digitalWrite(L_ENABLE_PIN, HIGH);
 Serial.println("电机已关闭使能（急停/省电/可手推）");
}


// ---------------------------------------------------------------------
// 5. 【重构】电机脉冲逻辑（由 Motor Task 调用）
// ---------------------------------------------------------------------
void executePulse(int stepPin, int dirPin, int enablePin, bool forward) {
 // 必须先使能，否则没力
 digitalWrite(enablePin, LOW); 
 // 设置方向
 digitalWrite(dirPin, forward ? HIGH : LOW);
 // 脉冲序列
 digitalWrite(stepPin, HIGH);     
 ets_delay_us(5); // 使用ets_delay_us替代delayMicroseconds，减少调度器干预
 digitalWrite(stepPin, LOW);
}


// ---------------------------------------------------------------------
// 6. 【核心】Motor Task - FreeRTOS 任务函数 (已修复队列接收参数)
// ---------------------------------------------------------------------
void motor_task(void *pvParameters) {
  MotorCommand_t currentCmd;

  Serial.println("Motor Task: 任务已启动，等待 BLE 指令...");

  // 任务的无限循环
  while (1) {
    // 尝试从队列中读取指令，等待时间设置为 100 毫秒
    // 之前使用 portMAX_DELAY 导致了断言错误，现在使用有限等待
    if (xQueueReceive(motorCommandQueue, &currentCmd, pdMS_TO_TICKS(100)) == pdTRUE) { 
      // 收到指令，开始执行
      switch (currentCmd) {
        case CMD_RP1:
          Serial.println("Motor Task: 执行 RP1 (右正)");
          executePulse(R_STEP_PIN, R_DIR_PIN, R_ENABLE_PIN, true);
          break;
        case CMD_RP0:
          Serial.println("Motor Task: 执行 RP0 (右反)");
          executePulse(R_STEP_PIN, R_DIR_PIN, R_ENABLE_PIN, false);
          break;
        case CMD_LP1:
          Serial.println("Motor Task: 执行 LP1 (左正)");
          executePulse(L_STEP_PIN, L_DIR_PIN, L_ENABLE_PIN, true);
          break;
        case CMD_LP0:
          Serial.println("Motor Task: 执行 LP0 (左反)");
          executePulse(L_STEP_PIN, L_DIR_PIN, L_ENABLE_PIN, false);
          break;
        case CMD_ENABLE_ON:
          enableMotors();
          break;
        case CMD_ENABLE_OFF:
          disableMotors();
          break;
        default:
          break;
      }
    }
    // 不需要额外的延迟，xQueueReceive已经会让出CPU时间
  }
}


// ---------------------------------------------------------------------
// 7. BLE 接收回调（将指令发送到队列，不再阻塞）
// ---------------------------------------------------------------------
class MyCallbacks : public BLECharacteristicCallbacks {
 void onWrite(BLECharacteristic* pChar) {
  auto val = pChar->getValue();
  String cmd = String(val.c_str());
  cmd.trim();
  if (cmd.length() == 0) return;
  cmd.toUpperCase();

  Serial.print("BLE 收到指令 → ");
  Serial.println(cmd);
   
  // 回显到客户端（如果已连接）
  if (deviceConnected) {
   pTxCharacteristic->setValue(cmd);
   pTxCharacteristic->notify();
  }

  MotorCommand_t commandToSend;
  bool sendToQueue = true;

  // 指令解析
  if (cmd == "RP1")   commandToSend = CMD_RP1;
  else if (cmd == "RP0") commandToSend = CMD_RP0;
  else if (cmd == "LP1") commandToSend = CMD_LP1;
  else if (cmd == "LP0") commandToSend = CMD_LP0;
  else if (cmd == "ENABLE_ON") commandToSend = CMD_ENABLE_ON;
  else if (cmd == "ENABLE_OFF" || cmd == "STOP" || cmd == "ESTOP") {
   commandToSend = CMD_ENABLE_OFF; 
   Serial.println("紧急停止触发！");
  }
  else if (cmd == "TEST" || cmd == "PING") {
   Serial.println("PONG → 设备正常运行");
   sendToQueue = false; // 不需要发送到电机队列
  } else {
   sendToQueue = false; // 未知指令
  }

  // 将指令发送到电机任务队列（异步操作，不阻塞）
  if (sendToQueue) {
   if (xQueueSend(motorCommandQueue, &commandToSend, 0) != pdPASS) {
    Serial.println("警告: 队列已满，指令丢失。");
   }
  }
 }
};

// ---------------------------------------------------------------------
// BLE 连接/断开回调（用于断线重连）
// ---------------------------------------------------------------------
class ServerCallbacks : public BLEServerCallbacks {
 void onConnect(BLEServer* pServer) {
  deviceConnected = true;
  Serial.println("Client Connected.");
 }
 void onDisconnect(BLEServer* pServer) {
  deviceConnected = false;
  Serial.println("Client Disconnected.");
 }
};


//======================================================================
// PART 3: CameraWebServer.ino 原始 setup() 函数的修改和整合
//======================================================================

void setup() {
 Serial.begin(115200);
 Serial.setDebugOutput(true);
 Serial.println();

 // ---------------------------------------------------------------------
 // A. CameraWebServer 原始的摄像头初始化逻辑
 // ---------------------------------------------------------------------
 camera_config_t config;
 config.ledc_channel = LEDC_CHANNEL_0;
 // ... (省略相机配置代码，保持不变) ...
  
 // 初始化摄像头
 esp_err_t err = esp_camera_init(&config);
 if (err != ESP_OK) {
  Serial.printf("Camera init failed with error 0x%x", err);
  return;
 }
 // ... (省略传感器配置代码，保持不变) ...
  
// 启动 WiFi
WiFi.begin(ssid, password);
Serial.print("Connecting to WiFi");

// 最多等待 20 秒 (20 * 500ms)
int maxAttempts = 40; 
while (WiFi.status() != WL_CONNECTED && maxAttempts > 0) {
    delay(500);
    Serial.print(".");
    maxAttempts--;
}

Serial.println("");

if (WiFi.status() == WL_CONNECTED) {
    // 连接成功
    Serial.println("WiFi connected SUCCESS!");
    Serial.printf("Starting stream server on %s\n", WiFi.localIP().toString().c_str());
    startCameraServer();
} else {
    // 连接失败
    Serial.println("WiFi connection FAILED. Please check SSID/Password.");
    // 如果连接失败，可以让程序在这里等待，防止后续功能继续运行
    while (true) { 
        delay(1000); 
    }
}
setupLedFlash();
  
 // ---------------------------------------------------------------------
 // B. 【新增】电机引脚初始化和 FreeRTOS 任务启动
 // ---------------------------------------------------------------------
 Serial.println("\n--- Motor/BLE Initialization ---");
 // 初始化所有电机引脚并默认关闭使能
 pinMode(R_STEP_PIN,  OUTPUT); digitalWrite(R_STEP_PIN,  LOW);
 pinMode(R_DIR_PIN,   OUTPUT); digitalWrite(R_DIR_PIN,   LOW);
 pinMode(R_ENABLE_PIN, OUTPUT); digitalWrite(R_ENABLE_PIN, HIGH); // 高电平=关闭使能
  
 pinMode(L_STEP_PIN,  OUTPUT); digitalWrite(L_STEP_PIN,  LOW);
 pinMode(L_DIR_PIN,   OUTPUT); digitalWrite(L_DIR_PIN,   LOW);
 pinMode(L_ENABLE_PIN, OUTPUT); digitalWrite(L_ENABLE_PIN, HIGH); // 高电平=关闭使能

 // 1. 创建 Motor Command 队列
 // 队列大小设为 10，允许最多缓冲 10 个电机指令
 motorCommandQueue = xQueueCreate(10, sizeof(MotorCommand_t)); 
 if (motorCommandQueue == NULL) {
  Serial.println("错误: 无法创建 Motor Command 队列!");
  return;
 }
  
 // 2. 启动 Motor FreeRTOS 任务
 // 将任务分配到 Core 1，避免与摄像头和PSRAM冲突
 xTaskCreatePinnedToCore(
   motor_task,     // 任务函数
   "MotorTask",    // 任务名称
   8192,        // 堆栈大小 (Bytes)，增加到8192字节防止溢出
   NULL,        // 任务参数
   0,         // 任务优先级，降低到最低优先级
   NULL,        // 任务句柄
   1);         // 绑定到 Core 1

 // ---------------------------------------------------------------------
 // C. 【新增】BLE 初始化逻辑
 // ---------------------------------------------------------------------
 BLEDevice::init("ESP32_Camera_Motor"); // 蓝牙名字
 BLEServer* pServer = BLEDevice::createServer();
 pServer->setCallbacks(new ServerCallbacks()); // 设置连接回调
 BLEService* pService = pServer->createService(SERVICE_UUID);

 // 创建“发送”特征（Notify）
 pTxCharacteristic = pService->createCharacteristic(
   TX_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
 pTxCharacteristic->addDescriptor(new BLE2902()); // 必须加这个才能Notify

 // 创建“接收”特征（Write）
 BLECharacteristic* pRx = pService->createCharacteristic(
   RX_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
 pRx->setCallbacks(new MyCallbacks());

 // 启动服务和广播
 pService->start();
 BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
 pAdvertising->addServiceUUID(SERVICE_UUID);
 pAdvertising->setScanResponse(true);
 pAdvertising->start();

 // 开机自检：使能200ms
 Serial.println("开机自检：使能电机200ms...");
 enableMotors();
 delay(200);
 disableMotors();
 Serial.println("初始化完成，系统处于安全状态（使能关闭）");
 Serial.println("==================================================");

} // end of setup()


//======================================================================
// PART 4: CameraWebServer.ino 原始 loop() 函数的修改和整合
//======================================================================

void loop() {
 // ---------------------------------------------------------------------
 // B. 【新增】BLE 串口转发逻辑
 // ---------------------------------------------------------------------
 // 把电脑串口监视器手动输入的内容也转发到蓝牙（方便你直接用串口调试）
 if (Serial.available()) {
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() > 0) {
   input += "\n";
   if (deviceConnected) {
    pTxCharacteristic->setValue(input);
    pTxCharacteristic->notify();
   }
  }
 }

 // ---------------------------------------------------------------------
 // C. 【新增】检测到手机/电脑断开连接后自动重新广播逻辑
 // ---------------------------------------------------------------------
 static bool lastConnected = false;
 if (!deviceConnected && lastConnected) {
  delay(500);
  BLEDevice::startAdvertising(); // 重新开始广播，等待新的连接
  Serial.println("蓝牙已断开，重新开始广播...");
 }
 lastConnected = deviceConnected; // 更新上一次连接状态

 // 主循环保持快速，避免阻塞
 delay(1); 
}

// ---------------------------------------------------------------------
// 原始 CameraWebServer.ino 文件的 startCameraServer() 和 setupLedFlash() 
// 函数放在文件末尾，内容保持不变。（您的代码应该有这两个函数，未显示在报错文件中）
// ---------------------------------------------------------------------
// startCameraServer()
// setupLedFlash()