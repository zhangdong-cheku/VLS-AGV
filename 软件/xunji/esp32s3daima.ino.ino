/*
ESP32S3 双步进电机独立控制程序（蓝牙版）
支持一条命令独立设置两个电机的方向、转速和步数
*/

// 引入BLE库
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE服务和特征UUID
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define RX_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // 接收数据
#define TX_CHARACTERISTIC_UUID "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"  // 发送数据

// BLE对象
BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
BLECharacteristic* pRxCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// BLE接收缓冲区（环形缓冲区，避免String操作的并发问题）

/*
什么是环形缓冲区？
环形缓冲区（也叫循环缓冲区）是一种 固定大小的、首尾相连的数据结构 ，专门用于 生产者-消费者模型 的数据传输。可以想象成：
数据写入（BLE回调函数 - 当蓝牙接收到数据时， 生产者（BLE回调函数） 会将数据写入缓冲区的 写入位置（bleBufferHead） ，并 移动写入位置 到下一个空位置。
数据读取（主循环 - 第185-189行）
主循环中， 消费者（readByte函数） 


- 一个 首尾相接的环形传送带 （固定长度）
- 一边有人 放东西 （生产者），一边有人 取东西 （消费者）
- 当传送带满了，新东西要么 替换旧东西 ，要么 停止放入
- 当传送带空了，就 没有东西可拿

第27行： volatile int bleBufferHead = 0;  // 写入位置（BLE回调写入）
- volatile ： 易变关键字 ，告诉编译器"这个变量随时可能被硬件或其他代码修改，不要优化它"
- int ：整数类型（4字节）
- bleBufferHead ：变量名，意为"BLE缓冲区的头部"，表示 新数据写入的位置
- = 0 ：初始值为0，从缓冲区的第一个位置开始写入
- 注释 ：这个位置由BLE回调函数（蓝牙数据到达时的处理函数）来更新 第28行： volatile int bleBufferTail = 0;  // 读取位置（主循环读取）
- bleBufferTail ：变量名，意为"BLE缓冲区的尾部"，表示 旧数据读取的位置
- = 0 ：初始值为0，从缓冲区的第一个位置开始读取
- 注释 ：这个位置由主循环（loop()函数）来更新 二、上下文：环形缓冲区的工作原理
这两个变量配合第26行的 char bleBuffer[BLE_BUFFER_SIZE]; ，组成了一个 环形缓冲区 （也叫循环缓冲区）：
把BLE缓冲区比作一个 快递中转站 ：

- bleBuffer ：中转站的 货仓 （容量256个包裹）
- bleBufferHead ：快递 入库口 的位置标记（新快递放这里）
- bleBufferTail ：快递 出库口 的位置标记（旧快递从这里取走）
- volatile ：因为入库和出库是 两个人同时操作 ，所以位置标记可能随时变化
实际工作流程 1. 数据写入（BLE回调）： 蓝牙接收到数据时，BLE回调函数将数据写入bleBuffer[bleBufferHead] 然后bleBufferHead向后移动一位（到下一个写入位置） 
如果到达缓冲区末尾（255），就回到开头（0），形成"环形"   2. 数据读取（主循环）： 主循环从bleBuffer[bleBufferTail]读取数据 然后bleBufferTail向后移动一位（到下一个读取位置） 
同样，如果到达末尾就回到开头   3. 空/满判断： 当head == tail时：缓冲区为空（没有新数据） 当(head + 1) % BLE_BUFFER_SIZE == tail时：缓冲区已满（不能再写入）*/
#define BLE_BUFFER_SIZE 256
char bleBuffer[BLE_BUFFER_SIZE];
volatile int bleBufferHead = 0;  // 写入位置（BLE回调写入）bleBufferHead：变量名，意为"BLE缓冲区的头部"，表示新数据写入的位置
volatile int bleBufferTail = 0;  // 读取位置（主循环读取）bleBufferTail ：变量名，意为"BLE缓冲区的尾部"，表示 旧数据读取的位置

// BLE设备名称
#define BLE_DEVICE_NAME "ESP32_Stepper_BLE"

// 电机引脚定义
#define R_STEP_PIN 4    // 右电机步进脉冲引脚
#define R_DIR_PIN 5     // 右电机方向控制引脚
#define R_ENABLE_PIN 6  // 右电机使能引脚（低电平使能）
#define L_STEP_PIN 12   // 左电机步进脉冲引脚
#define L_DIR_PIN 19    // 左电机方向控制引脚
#define L_ENABLE_PIN 14 // 左电机使能引脚（低电平使能）

// 步进延迟时间（微秒），控制电机速度
#define STEP_DELAY 100

// 电机方向设置
#define R_FORWARD LOW  // 右电机正转方向
#define R_BACKWARD HIGH  // 右电机反转方向
#define L_FORWARD HIGH   // 左电机正转方向
#define L_BACKWARD LOW // 左电机反转方向

// 电机状态结构体
typedef struct {
  boolean isRunning;    // 电机运行状态
  byte direction;       // 当前方向
  int remainingSteps;   // 剩余步数
  unsigned long lastStepTime; // 上一步时间
  int stepPin;          // 步进引脚
  int dirPin;           // 方向引脚
  boolean stepState;    // 当前步进引脚状态（HIGH/LOW）
  boolean isContinuousRunning; // 持续运行模式标志
  unsigned long stepDelay; // 步进延迟时间（微秒），控制电机速度
} MotorState;

// 初始化左右电机状态
MotorState rightMotor = {false, R_FORWARD, 0, 0, R_STEP_PIN, R_DIR_PIN, LOW, false, STEP_DELAY};
MotorState leftMotor = {false, L_FORWARD, 0, 0, L_STEP_PIN, L_DIR_PIN, LOW, false, STEP_DELAY};

// 命令缓冲区
String receivedCommand = "";
String lastProcessedCommand = "";  // 记录最后处理的命令，防止重复执行
unsigned long lastCommandTime = 0; // 最后命令时间
#define COMMAND_DEBOUNCE_MS 100    // 命令去重时间窗口（毫秒）

// 调试模式开关（设为true可以看到详细的接收数据）
#define DEBUG_MODE false

// BLE服务器回调类
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      #if DEBUG_MODE
      Serial.println("BLE客户端已连接");
      #endif
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      #if DEBUG_MODE
      Serial.println("BLE客户端已断开");
      #endif
    }
};

// BLE接收特征回调类
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue().c_str();
      if (value.length() > 0) {
        #if DEBUG_MODE
        // 调试：显示原始接收数据
        Serial.print("[BLE接收] 长度:");
        Serial.print(value.length());
        Serial.print(" 内容:'");
        Serial.print(value);
        Serial.print("' HEX:");
        for (size_t i = 0; i < value.length(); i++) {
          Serial.print(" 0x");
          Serial.print((byte)value[i], HEX);
        }
        Serial.println();
        #endif
        
        // 将接收到的数据写入环形缓冲区
        for (size_t i = 0; i < value.length(); i++) {
          int nextHead = (bleBufferHead + 1) % BLE_BUFFER_SIZE;
          
          // 检查缓冲区是否已满
          if (nextHead != bleBufferTail) {
            bleBuffer[bleBufferHead] = value[i];
            bleBufferHead = nextHead;
          } else {
            #if DEBUG_MODE
            Serial.println("[警告] BLE缓冲区已满，丢弃数据！");
            #endif
            break;
          }
        }
        
        #if DEBUG_MODE
        // 计算缓冲区中的数据量
        int bufferSize = (bleBufferHead - bleBufferTail + BLE_BUFFER_SIZE) % BLE_BUFFER_SIZE;
        Serial.print("[缓冲区] 当前数据量:");
        Serial.print(bufferSize);
        Serial.println(" 字节");
        #endif
      }
    }
};

// BLE辅助函数：同时向串口和BLE发送消息
// 添加一个简单的串口输出检查，避免在无串口连接时阻塞
void printToBoth(String message) {
  // 仅在DEBUG_MODE下且有数据发送可能时才输出到串口
  // 避免无串口连接时的阻塞
  #if DEBUG_MODE
  Serial.println(message);
  #endif
  if (deviceConnected && pTxCharacteristic != NULL) {
    String msg = message + "\n";
    pTxCharacteristic->setValue(msg);
    pTxCharacteristic->notify();
  }
}

void printToBoth(const char* message) {
  // 仅在DEBUG_MODE下且有数据发送可能时才输出到串口
  // 避免无串口连接时的阻塞
  #if DEBUG_MODE
  Serial.println(message);
  #endif
  if (deviceConnected && pTxCharacteristic != NULL) {
    String msg = String(message) + "\n";
    pTxCharacteristic->setValue(msg);
    pTxCharacteristic->notify();
  }
}

// 检查是否有可用数据（串口或BLE）
int availableData() {
  if (Serial.available() > 0) return 1;  // 串口数据
  if (bleBufferTail != bleBufferHead) return 2;  // BLE数据
  return 0; // 无数据
}

// 从可用源读取一个字节
char readByte() {
  if (Serial.available() > 0) {
    return Serial.read();
  }
  // 从环形缓冲区读取
  if (bleBufferTail != bleBufferHead) {
    char c = bleBuffer[bleBufferTail];
    bleBufferTail = (bleBufferTail + 1) % BLE_BUFFER_SIZE;
    return c;
  }
  return 0;
}

// 电机单步控制函数 - 完全非阻塞式（状态机方式）
void updateMotor(MotorState &motor) {
  // 运行条件：电机正在运行且（有剩余步数 或 处于持续运行模式）
  if (!motor.isRunning || (!motor.isContinuousRunning && motor.remainingSteps <= 0)) {
    return;
  }
  
  unsigned long currentTime = micros();
  
  // 状态机：HIGH 和 LOW 状态交替
  if (motor.stepState == LOW) {
    // 当前是LOW状态，检查是否该切换到HIGH
    if (currentTime - motor.lastStepTime >= motor.stepDelay) {
      digitalWrite(motor.stepPin, HIGH);
      motor.stepState = HIGH;
      motor.lastStepTime = currentTime;
    }
  } else {
    // 当前是HIGH状态，检查是否该切换到LOW（完成一步）
    if (currentTime - motor.lastStepTime >= motor.stepDelay) {
      digitalWrite(motor.stepPin, LOW);
      motor.stepState = LOW;
      motor.lastStepTime = currentTime;
      
      // 非持续运行模式下才更新剩余步数和检查完成
      if (!motor.isContinuousRunning) {
        motor.remainingSteps--;  // 完成一步
        
        // 检查是否完成所有步数
        if (motor.remainingSteps <= 0) {
          motor.isRunning = false;
          String msg = String(motor.stepPin == R_STEP_PIN ? "右" : "左") + "电机运动完成";
          printToBoth(msg);
        }
      }
    }
  }
}

// 显示帮助信息
void showHelp() {
  printToBoth("========================================");
  printToBoth("ESP32 双步进电机独立控制系统");
  printToBoth("========================================");
  printToBoth("");
  printToBoth("【主要功能】");
  printToBoth("一条命令独立控制两个电机的方向和步数");
  printToBoth("");
  printToBoth("【命令格式】");
  printToBoth("BOTH<右方向><右步数>,<左方向><左步数>");
  printToBoth("");
  printToBoth("【参数说明】");
  printToBoth("- 方向: F=正转, B=反转");
  printToBoth("- 步数: 任意正整数");
  printToBoth("- 必须用逗号分隔左右电机参数");
  printToBoth("");
  printToBoth("【命令示例】");
  printToBoth("BOTHF500,B1000  - 右电机正转500步, 左电机反转1000步");
  printToBoth("BOTHB800,F600   - 右电机反转800步, 左电机正转600步");
  printToBoth("BOTHF1000,F500  - 右电机正转1000步, 左电机正转500步");
  printToBoth("BOTHB300,B800   - 右电机反转300步, 左电机反转800步");
  printToBoth("");
  printToBoth("【单个电机控制】");
  printToBoth("RF500   - 右电机正转500步");
  printToBoth("RB1000  - 右电机反转1000步");
  printToBoth("LF500   - 左电机正转500步");
  printToBoth("LB1000  - 左电机反转1000步");
  printToBoth("");
  printToBoth("【其他命令】");
  printToBoth("F       - 持续前进（两个电机同时持续前进）");
  printToBoth("T       - 持续后退（两个电机同时持续后退）");
  printToBoth("TL      - 左转（左电机不转，右侧电机前进）");
  printToBoth("TR      - 右转（右电机不转，左侧电机前进）");
  printToBoth("S       - 停止所有电机");
  printToBoth("SPEED   - 设置电机速度（1-100%）");
  printToBoth("  格式: SPEED<电机><速度> 或 SPEED<电机><速度>,<电机><速度>");
  printToBoth("  示例: SPEEDR50      - 右电机速度设为50%");
  printToBoth("        SPEEDL70,R30  - 左电机70%, 右电机30%");
  printToBoth("        SPEEDB100     - 双电机速度设为100%");
  printToBoth("H       - 显示此帮助信息");
  printToBoth("");
  printToBoth("【速度说明】");
  printToBoth("- 速度值范围: 1-100%");
  printToBoth("- 速度越高，步进延迟越小，电机转得越快");
  printToBoth("- 默认速度: 约33%（300微秒延迟）");
  printToBoth("========================================");
  printToBoth("【注意】");
  printToBoth("使用F或T命令后，电机将持续运行，直到收到S命令停止");
}

// 解析并执行命令
void processCommand(String command) {
  command.trim();  // 去除首尾空格
  
  // 检查命令是否为空
  if (command.length() == 0) {
   // 这是 条件编译指令（仅在编译阶段生效）：
   //若定义了 DEBUG_MODE（通常通过 #define DEBUG_MODE 声明），编译时会保留这条日志打印语句，通过串口输出调试信息，方便开发时排查问题
   //若未定义 DEBUG_MODE（发布版本常用），这条语句会被编译器剔除，不占用运行资源、不输出冗余信息
    #if DEBUG_MODE
    Serial.println("[命令处理] 空命令，忽略");
    #endif
    return;
  }
  
  //command.toUpperCase() 不会是嵌入式底层（如 C 语言）的原生写法，而是 嵌入式上层 / 框架化开发 中的用法 —— 核心仍是 “字符串小写转大写”，但依赖具体的嵌入式语言、RTOS 或框架，而非 C 语言标准库。
  command.toUpperCase(); // 转换为大写
  
  // 命令去重：防止重复命令在短时间内多次执行

  //这行代码是 Arduino（或兼容 AVR/ARM 单片机） 中的常用写法，核心作用是获取系统运行后的 “毫秒级累计时间”，简要解析如下：
  ///1. 变量类型：unsigned long
  //无符号长整型（32 位），取值范围 0 ~ 4294967295（约 49.7 天）；
  //选择该类型是因为：millis() 的返回值本身是 unsigned long，且无符号能避免负数溢出问题（时间只会递增，溢出后从 0 重新计数）。
  //. 函数：millis()
  //核心功能：返回单片机上电（或复位）后累计的 毫秒数（1 秒 = 1000 毫秒）；
  //特性：
  //非阻塞（不暂停程序），底层由定时器中断驱动，程序运行时会自动累计；
  //溢出周期：约 49.7 天（32 位无符号数的最大值 ÷ 1000 毫秒 / 秒 ≈ 49.7 天），溢出后从 0 开始重新计数。
  //3. 整体作用
  //将当前系统累计运行时间（毫秒级）存储到 currentTime 变量中，常用于：
  //计时（如 “延时 N 毫秒后执行某个操作”，替代阻塞的 delay() 函数）；
  //记录事件发生时间（如按键按下、传感器触发的时间戳）；
  //计算时间间隔（如两次操作的时间差）。
  unsigned long currentTime = millis();

  if (command == lastProcessedCommand && 
      (currentTime - lastCommandTime) < COMMAND_DEBOUNCE_MS) {
    #if DEBUG_MODE
    Serial.print("[命令去重] 忽略重复命令: ");
    Serial.print(command);
    Serial.print(" (距上次仅");
    Serial.print(currentTime - lastCommandTime);
    Serial.println("ms)");
    #endif
    return;
  }
  
  // 更新最后处理的命令和时间
  lastProcessedCommand = command;
  lastCommandTime = currentTime;
  
  // 打印接收到的命令
  printToBoth("接收到命令: " + command);
  
  // 帮助命令
  if (command == "H" || command == "HELP") {
    showHelp();
    return;
  }
  
  // 停止命令
  if (command == "S" || command == "STOP") {
    printToBoth("执行停止命令，所有电机停止运动");
    rightMotor.isRunning = false;
    leftMotor.isRunning = false;
    rightMotor.remainingSteps = 0;
    leftMotor.remainingSteps = 0;
    rightMotor.isContinuousRunning = false;
    leftMotor.isContinuousRunning = false;
    return;
  }
  
  // 持续前进命令
  if (command == "F" || command == "FORWARD") {
    printToBoth("执行持续前进命令，两个电机同时前进");
    
    // 设置右电机
    rightMotor.direction = R_FORWARD;
    rightMotor.remainingSteps = 1;  // 设为任意正整数，持续运行模式下不检查
    rightMotor.lastStepTime = micros();
    rightMotor.stepState = LOW;
    rightMotor.isRunning = true;
    rightMotor.isContinuousRunning = true;
    digitalWrite(R_DIR_PIN, rightMotor.direction);
    digitalWrite(R_STEP_PIN, LOW);
    
    // 设置左电机
    leftMotor.direction = L_FORWARD;
    leftMotor.remainingSteps = 1;  // 设为任意正整数，持续运行模式下不检查
    leftMotor.lastStepTime = micros();
    leftMotor.stepState = LOW;
    leftMotor.isRunning = true;
    leftMotor.isContinuousRunning = true;
    digitalWrite(L_DIR_PIN, leftMotor.direction);
    digitalWrite(L_STEP_PIN, LOW);
    
    printToBoth("✓ 两个电机开始持续前进");
    return;
  }
  
  // 持续后退命令
  if (command == "T" || command == "BACKWARD") {
    printToBoth("执行持续后退命令，两个电机同时后退");
    
    // 设置右电机
    rightMotor.direction = R_BACKWARD;
    rightMotor.remainingSteps = 1;  // 设为任意正整数，持续运行模式下不检查
    rightMotor.lastStepTime = micros();
    rightMotor.stepState = LOW;
    rightMotor.isRunning = true;
    rightMotor.isContinuousRunning = true;
    digitalWrite(R_DIR_PIN, rightMotor.direction);
    digitalWrite(R_STEP_PIN, LOW);
    
    // 设置左电机
    leftMotor.direction = L_BACKWARD;
    leftMotor.remainingSteps = 1;  // 设为任意正整数，持续运行模式下不检查
    leftMotor.lastStepTime = micros();
    leftMotor.stepState = LOW;
    leftMotor.isRunning = true;
    leftMotor.isContinuousRunning = true;
    digitalWrite(L_DIR_PIN, leftMotor.direction);
    digitalWrite(L_STEP_PIN, LOW);
    
    printToBoth("✓ 两个电机开始持续后退");
    return;
  }
  
  // 左转命令（左电机不转，右侧电机前进）
  if (command == "TL" || command == "TURNLEFT") {
    printToBoth("执行左转命令，左电机不转，右侧电机前进");
    
    // 设置右电机前进
    rightMotor.direction = R_FORWARD;
    rightMotor.remainingSteps = 1;  // 设为任意正整数，持续运行模式下不检查
    rightMotor.lastStepTime = micros();
    rightMotor.stepState = LOW;
    rightMotor.isRunning = true;
    rightMotor.isContinuousRunning = true;
    digitalWrite(R_DIR_PIN, rightMotor.direction);
    digitalWrite(R_STEP_PIN, LOW);
    
    // 确保左电机停止
    leftMotor.isRunning = false;
    leftMotor.remainingSteps = 0;
    leftMotor.isContinuousRunning = false;
    
    printToBoth("✓ 已执行左转命令");
    return;
  }
  
  // 右转命令（右电机不转，左侧电机前进）
  if (command == "TR" || command == "TURNRIGHT") {
    printToBoth("执行右转命令，右电机不转，左侧电机前进");
    
    // 设置左电机前进
    leftMotor.direction = L_FORWARD;
    leftMotor.remainingSteps = 1;  // 设为任意正整数，持续运行模式下不检查
    leftMotor.lastStepTime = micros();
    leftMotor.stepState = LOW;
    leftMotor.isRunning = true;
    leftMotor.isContinuousRunning = true;
    digitalWrite(L_DIR_PIN, leftMotor.direction);
    digitalWrite(L_STEP_PIN, LOW);
    
    // 确保右电机停止
    rightMotor.isRunning = false;
    rightMotor.remainingSteps = 0;
    rightMotor.isContinuousRunning = false;
    
    printToBoth("✓ 已执行右转命令");
    return;
  }
  
  // BOTH命令 - 独立控制两个电机
  if (command.startsWith("BOTH")) {
  
  int commaPos = command.indexOf(',');
    
    if (commaPos <= 4) {
      printToBoth("命令格式错误！");
      printToBoth("格式: BOTH<右方向><右步数>,<左方向><左步数>");
      printToBoth("例如: BOTHF500,B1000");
      return;
    }
    
    // 解析右电机参数
    String rightPart = command.substring(4, commaPos);
    if (rightPart.length() < 2) {
      printToBoth("右电机参数格式错误");
      return;
    }
   // 获取字符串中指定索引位置的字符：
   //索引从 0 开始（字符串的第一个字符对应索引 0，第二个对应 1，以此类推）；
   //charAt(0) 即明确获取 “字符串的第一个字符”
    char rightDir = rightPart.charAt(0);
    int rightSteps = rightPart.substring(1).toInt();
    
    // 解析左电机参数
    String leftPart = command.substring(commaPos + 1);
    if (leftPart.length() < 2) {
      printToBoth("左电机参数格式错误");
      return;
    }
    char leftDir = leftPart.charAt(0);
    int leftSteps = leftPart.substring(1).toInt();
    
    // 验证参数
    if ((rightDir != 'F' && rightDir != 'B') || (leftDir != 'F' && leftDir != 'B')) {
      printToBoth("方向标识错误，应为F(正转)或B(反转)");
      return;
    }
    
    if (rightSteps <= 0 || leftSteps <= 0) {
      printToBoth("步数必须为正整数");
      return;
    }
    
    // 显示控制信息
    String rightDirStr = (rightDir == 'F') ? "正转" : "反转";
    String leftDirStr = (leftDir == 'F') ? "正转" : "反转";
    printToBoth("右电机" + rightDirStr + String(rightSteps) + "步, 左电机" + leftDirStr + String(leftSteps) + "步");
    
    // 设置右电机
    rightMotor.direction = (rightDir == 'F') ? R_FORWARD : R_BACKWARD;
    rightMotor.remainingSteps = rightSteps;
    rightMotor.lastStepTime = micros();
    rightMotor.stepState = LOW;  // 初始化为LOW状态
    rightMotor.isRunning = true;
    digitalWrite(R_DIR_PIN, rightMotor.direction);
    digitalWrite(R_STEP_PIN, LOW);  // 确保初始为LOW
    
    // 设置左电机
    leftMotor.direction = (leftDir == 'F') ? L_FORWARD : L_BACKWARD;
    leftMotor.remainingSteps = leftSteps;
    leftMotor.lastStepTime = micros();
    leftMotor.stepState = LOW;  // 初始化为LOW状态
    leftMotor.isRunning = true;
    digitalWrite(L_DIR_PIN, leftMotor.direction);
    digitalWrite(L_STEP_PIN, LOW);  // 确保初始为LOW
    
    printToBoth("✓ 两个电机已开始独立运行");
    return;
  }
  
  // 单个电机控制
  if (command.length() >= 3) {
    //charAt(0)：字符串的内置方法，作用是获取字符串中指定索引位置的字符。索引从 0 开始（第一个字符索引为 0，第二个为 1，以此类推），所以 charAt(0) 就是取字符串的 “第一个字符”；
    char motor = command.charAt(0);
    char direction = command.charAt(1);
    
    if ((motor == 'R' || motor == 'L') && (direction == 'F' || direction == 'B')) {
      String stepsStr = command.substring(2);
      int steps = stepsStr.toInt();
      
      if (steps <= 0) {
        printToBoth("步数必须为正整数");
        return;
      }
      
      if (motor == 'R') {
        String msg = "右电机" + String(direction == 'F' ? "正转" : "反转") + String(steps) + "步";
        printToBoth(msg);
        
        rightMotor.direction = (direction == 'F') ? R_FORWARD : R_BACKWARD;
        rightMotor.remainingSteps = steps;
        rightMotor.lastStepTime = micros();
        rightMotor.stepState = LOW;
        rightMotor.isRunning = true;
        digitalWrite(R_DIR_PIN, rightMotor.direction);
        digitalWrite(R_STEP_PIN, LOW);
        
      } else if (motor == 'L') {
        String msg = "左电机" + String(direction == 'F' ? "正转" : "反转") + String(steps) + "步";
        printToBoth(msg);
        
        leftMotor.direction = (direction == 'F') ? L_FORWARD : L_BACKWARD;
        leftMotor.remainingSteps = steps;
        leftMotor.lastStepTime = micros();
        leftMotor.stepState = LOW;
        leftMotor.isRunning = true;
        digitalWrite(L_DIR_PIN, leftMotor.direction);
        digitalWrite(L_STEP_PIN, LOW);
      }
      
      printToBoth("✓ 命令已设置");
      return;
    }
  }
  
  // 速度控制命令
  if (command.startsWith("SPEED")) {
    // 解析速度命令：SPEED<电机><速度> 或 SPEED<电机><速度>,<电机><速度>
    String speedCommand = command.substring(5); // 去掉"SPEED"
    
    // 检查是否有逗号分隔的多个参数
    int commaPos = speedCommand.indexOf(',');
    
    if (commaPos > 0) {
      // 多个速度参数
      String firstPart = speedCommand.substring(0, commaPos);
      String secondPart = speedCommand.substring(commaPos + 1);
      
      // 处理第一个参数
      if (firstPart.length() >= 2) {
        char motor1 = firstPart.charAt(0);
        int speed1 = firstPart.substring(1).toInt();
        processSpeedCommand(motor1, speed1);
      }
      
      // 处理第二个参数
      if (secondPart.length() >= 2) {
        char motor2 = secondPart.charAt(0);
        int speed2 = secondPart.substring(1).toInt();
        processSpeedCommand(motor2, speed2);
      }
    } else {
      // 单个速度参数
      if (speedCommand.length() >= 2) {
        char motor = speedCommand.charAt(0);
        int speed = speedCommand.substring(1).toInt();
        processSpeedCommand(motor, speed);
      }
    }
    
    return;
  }
  
  // 未知命令
  printToBoth("未知命令，输入 H 查看帮助信息");
}

// 处理单个电机速度命令
void processSpeedCommand(char motor, int speed) {
  // 速度范围限制：1-100
  if (speed < 1 || speed > 100) {
    printToBoth("速度值必须在1-100之间");
    return;
  }
  
  // 速度转换为延迟时间（微秒）：速度越高，延迟越小
  // 默认100微秒对应100%速度，最大1000微秒对应10%速度
  unsigned long delayTime = map(speed, 1, 100, 1000, 100);
  
  String motorName = "";
  
  if (motor == 'R') {
    // 设置右电机速度
    rightMotor.stepDelay = delayTime;
    motorName = "右";
  } else if (motor == 'L') {
    // 设置左电机速度
    leftMotor.stepDelay = delayTime;
    motorName = "左";
  } else if (motor == 'B') {
    // 设置双电机速度
    rightMotor.stepDelay = delayTime;
    leftMotor.stepDelay = delayTime;
    motorName = "双";
  } else {
    printToBoth("电机类型错误，应为R(右)、L(左)或B(双)");
    return;
  }
  
  String msg = motorName + "电机速度已设置为" + String(speed) + "% (延迟: " + String(delayTime) + "微秒)";
  printToBoth(msg);
}

// 初始化系统
void setup() {
  // 设置串口通信
  Serial.begin(115200);
  #if DEBUG_MODE
  Serial.println("ESP32S3 双步进电机独立控制系统启动中...");
  #endif
  
  // 初始化BLE设备
  BLEDevice::init(BLE_DEVICE_NAME);
  
  // 创建BLE服务器
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // 创建BLE服务
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // 创建TX特征（用于发送数据给客户端）
  pTxCharacteristic = pService->createCharacteristic(
                        TX_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());
  
  // 创建RX特征（用于接收来自客户端的数据）
  pRxCharacteristic = pService->createCharacteristic(
                        RX_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_WRITE
                      );
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  
  // 启动服务
  pService->start();
  
  // 启动广播
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();    // 获取BLE广播控制器指针（广告宣传车的遥控器）
  pAdvertising->addServiceUUID(SERVICE_UUID);                     // 添加服务UUID到广播（在广告中说明提供步进电机控制服务）
  pAdvertising->setScanResponse(true);                            // 启用扫描响应（允许回应客户端的询问）
  pAdvertising->setMinPreferred(0x06);                            // 设置最小BLE版本偏好（优先BLE 4.0以上版本）
  pAdvertising->setMinPreferred(0x12);                            // 设置最小电源等级偏好（优先稳定电源连接）
  BLEDevice::startAdvertising();                                  // 开始BLE广播（广告宣传车出发，开始广播设备信息）
  
  #if DEBUG_MODE
  Serial.println("BLE服务已启动，等待连接...");
  Serial.println("设备名称: " + String(BLE_DEVICE_NAME));
  #endif
  
  // 设置电机控制引脚为输出
  pinMode(R_STEP_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);
  pinMode(R_ENABLE_PIN, OUTPUT);
  pinMode(L_STEP_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(L_ENABLE_PIN, OUTPUT);
  
  // 启用电机（低电平使能）
  digitalWrite(R_ENABLE_PIN, LOW);
  digitalWrite(L_ENABLE_PIN, LOW);
  
  #if DEBUG_MODE
  Serial.println("========================================");
  Serial.println("ESP32S3 双步进电机独立控制系统就绪");
  Serial.println("========================================");
  Serial.println("输入 H 查看帮助信息");
  Serial.println("========================================");
  #endif
}

// 主循环
void loop() {
  // 检查串口或蓝牙是否有数据到达
  while (availableData() > 0) {
    char incomingByte = readByte();
    
    #if DEBUG_MODE
    // 调试：显示接收到的每个字符
    if (incomingByte == '\n') {
      Serial.println("[接收字符] \\n (换行)");
    } else if (incomingByte == '\r') {
      Serial.println("[接收字符] \\r (回车)");
    } else if (incomingByte >= 32 && incomingByte <= 126) {
      Serial.print("[接收字符] '");
      Serial.print(incomingByte);
      Serial.println("'");
    } else {
      Serial.print("[接收字符] 0x");
      Serial.println((byte)incomingByte, HEX);
    }
    #endif
    
    // 如果收到换行符或回车符，处理完整命令
    if (incomingByte == '\n' || incomingByte == '\r') {
      if (receivedCommand.length() > 0) {
        #if DEBUG_MODE
        Serial.print("[处理命令] 长度:");
        Serial.print(receivedCommand.length());
        Serial.print(" 内容:'");
        Serial.print(receivedCommand);
        Serial.println("'");
        #endif
        
        processCommand(receivedCommand);
        receivedCommand = "";
      }
    } else {
      // 过滤掉不可见的控制字符（除了换行回车）
      if (incomingByte >= 32 && incomingByte <= 126) {
        receivedCommand += incomingByte;
      } else {
        #if DEBUG_MODE
        Serial.print("[忽略字符] 0x");
        Serial.println((byte)incomingByte, HEX);
        #endif
      }
    }
  }
  
  // 更新两个电机的状态 - 非阻塞方式
  updateMotor(rightMotor);
  updateMotor(leftMotor);
  
  // BLE断开重连处理

  /*左侧 !deviceConnected：deviceConnected 表示「当前设备是否连接」
  （布尔值：true = 连接，false = 未连接），! 是逻辑非，所以整体意思是「当前设备未连接」。
  右侧 oldDeviceConnected：oldDeviceConnected 表示「上一次状态中设备是否连接」，
  整体意思是「上一次设备是连接的」。
  逻辑与（&&）：只有左右两侧同时为 true 时，整个表达式才返回 true。*/
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);

    /* BLEServer：可以理解为"蓝牙服务器"的功能说明书，它定义了ESP32如何作为蓝牙服务器工作的所有规则和能力。 *：
    星号表示这是一个指针变量（可以理解为"地址簿"，用来存储某个"东西"的位置）。 pServer：这是给这个变量起的名字
    （类似地址簿的名称，比如"我家的地址簿"）。 = NULL：
    把这个地址簿初始化为空（表示这个地址簿里还没有写任何地址）*/
    pServer->startAdvertising();
    #if DEBUG_MODE
    Serial.println("开始广播，等待重连...");
    #endif
    oldDeviceConnected = deviceConnected;
  }
  
  // BLE连接成功处理
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    printToBoth("ESP32S3 双步进电机独立控制系统");
    printToBoth("输入 H 查看帮助信息");
  }
}
