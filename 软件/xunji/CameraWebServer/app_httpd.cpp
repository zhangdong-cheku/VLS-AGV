// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "esp32-hal-ledc.h"
#include "sdkconfig.h"
#include "camera_index.h"
#include "board_config.h"

/*这行代码是 条件编译指令，用于判断当前编译环境是否满足特定条件，只有满足条件时，后续的代码块才会被编译器处理。我们可以拆开来理解：
1. 整体结构：#if defined(A) && defined(B)
这是 C/C++ 中的预处理指令，意思是：当同时定义了宏 A 和宏 B 时，才执行后续代码（直到 #endif 结束）。这里的 A 是 ARDUINO_ARCH_ESP32，B 是 CONFIG_ARDUHAL_ESP_LOG。
2. 逐个解析宏的含义
（1）ARDUINO_ARCH_ESP32
这是 Arduino 框架 定义的宏，用于标识当前编译的目标硬件是 ESP32 系列芯片（包括 ESP32-S3、ESP32-C3 等）。
当你在 Arduino IDE 中选择 ESP32 开发板（如 ESP32S3 Dev Module）时，编译器会自动定义这个宏，告诉代码 “现在正在为 ESP32 芯片编译”。
（2）CONFIG_ARDUHAL_ESP_LOG
这是 ESP32 Arduino 核心库 中的一个配置宏，用于控制是否启用 ESP32 专用的日志系统（基于 ESP-IDF 的 esp_log 组件）。
该宏通常在 ESP32 开发板的配置文件中定义（如 sdkconfig.h），当启用日志功能时（默认启用），这个宏会被定义，允许代码使用 ESP_LOGI()、ESP_LOGE() 等日志函数。
3. 整行代码的作用
简单说，这行代码的意思是：“只有在 Arduino 框架下编译，且目标硬件是 ESP32 系列，同时启用了 ESP32 日志系统时，才执行后续的代码块”。
这段代码是条件性引入头文件的预处理指令，作用是：仅当当前编译环境是 ESP32 且启用了 ESP32 日志系统时，才引入 esp32-hal-log.h 头文件，否则跳过该头文件的引入。
逐句解析
#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)这是条件判断（和之前解释的含义一致）：
ARDUINO_ARCH_ESP32：确保当前编译目标是 ESP32 系列芯片（如 ESP32-S3）。
CONFIG_ARDUHAL_ESP_LOG：确保启用了 ESP32 的日志系统配置。
只有这两个条件同时满足，才会执行下一行代码。
#include "esp32-hal-log.h"这是引入头文件的指令。esp32-hal-log.h 是 ESP32 Arduino 核心库中的日志功能专用头文件，包含了 ESP_LOGI()（信息日志）、ESP_LOGE()（错误日志）、ESP_LOGD()（调试日志）等 ESP32 特有的日志函数声明。
#endif结束条件判断块，标志着 “仅在满足条件时执行” 的代码范围结束。
核心作用
避免跨平台编译错误：esp32-hal-log.h 是 ESP32 平台特有的头文件，其他 Arduino 硬件（如 Arduino UNO、ESP8266）没有这个文件。如果不做条件判断，直接引入该文件，在非 ESP32 平台编译时会报错 “找不到头文件”。
按需引入功能：只有当 ESP32 的日志系统启用时（CONFIG_ARDUHAL_ESP_LOG 被定义），引入该头文件才有意义，否则即使引入也无法使用日志函数。*/
#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

// LED FLASH setup
#if defined(LED_GPIO_NUM)
#define CONFIG_LED_MAX_INTENSITY 255

int led_duty = 0;
bool isStreaming = false;

#endif

typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

/*#define PART_BOUNDARY 这行代码是 C/C++ 中的宏定义语句，它定义了一个名为 PART_BOUNDARY 的宏，但没有为其指定具体的值（属于 “无值宏”）。
这行代码定义了一个静态常量字符串 _STREAM_CONTENT_TYPE，其值是 HTTP 协议中用于 “多部分混合替换”（multipart/x-mixed-replace）的 Content-Type 头部，并且通过宏 PART_BOUNDARY 拼接了分隔符（boundary）。
具体来说：
multipart/x-mixed-replace 是一种 HTTP 多部分内容类型，常用于流式传输（如视频、实时数据），服务器可以通过这种类型持续发送新的内容块，替换之前的内容。
boundary=PART_BOUNDARY 表示多部分内容中用于分隔不同块的边界标识，这里通过宏 PART_BOUNDARY 动态插入具体的边界字符串（例如 abc123）。
这行代码能正常工作的前提是，PART_BOUNDARY 必须是一个字符串常量宏，例如之前已经定义过：*/
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

/*这行代码声明了一个类型为 httpd_handle_t 的变量 stream_httpd，并将其初始化为 NULL。
从命名和上下文推测（结合之前的 HTTP 多部分内容类型相关代码），这很可能是在嵌入式 HTTP 服务器（如 ESP-IDF 中的 esp_http_server）中用于表示 “流服务 HTTP 服务器实例” 的句柄变量：
httpd_handle_t 通常是一个不透明指针类型（Opaque Pointer），用于标识和操作一个 HTTP 服务器实例（类似文件描述符的作用）。
初始化为 NULL 表示该服务器实例尚未创建或未初始化。
后续代码中可能会通过类似 stream_httpd = httpd_start(&config) 的方式启动 HTTP 服务器，并将返回的句柄赋值给 stream_httpd，用于后续的服务器操作（如注册 URI 处理函数、停止服务器等）。
这种写法在嵌入式 HTTP 服务开发中很常见，用于管理服务器实例的生命周期。*/

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

/*这段代码定义了一个名为 ra_filter_t 的结构体，并声明了一个该类型的静态变量 ra_filter，从结构成员来看，这很可能是一个用于滑动窗口滤波或均值滤波的过滤器数据结构。
结构体 ra_filter_t 成员解析：
size：类型为 size_t，注释说明为 “用于滤波的值的数量”，通常表示滤波窗口的大小（即参与一次滤波计算的样本总数）。
index：类型为 size_t，表示 “当前值的索引”，用于记录下一个待存储的样本在数组中的位置（通常在滑动窗口中循环使用）。
count：类型为 size_t，表示 “值的计数”，即当前已存储的样本数量（在窗口未填满时，count 小于 size；填满后等于 size）。
sum：类型为 int，存储当前已存储样本的总和，用于快速计算均值（避免每次滤波时重新求和，提升效率）。
values：类型为 int*，指向一个整数数组，用于存储实际的样本数据（数组大小通常由 size 决定）。
静态变量 ra_filter：
static ra_filter_t ra_filter; 声明了一个文件作用域的 ra_filter_t 类型变量，意味着它仅在当前 .c 文件中可见，通常用于在该文件内实现滤波逻辑的状态存储（如累计样本、计算均值等）。
典型用途（滑动窗口均值滤波）：
这种结构常用于对连续输入的整数数据（如传感器读数）进行平滑处理，核心逻辑大致如下：
初始化：设置 size（窗口大小），为 values 数组分配内存，初始化 index=0、count=0、sum=0。
添加新样本：
若窗口未满（count < size）：直接将新值存入 values[index]，sum 累加，count 递增。
若窗口已满：先用新值替换 values[index]（覆盖最旧的样本），sum 先减去被替换的值，再加上新值。
每次添加后，index 循环递增（如 index = (index + 1) % size）。
计算滤波结果：返回 sum / count（均值）。
通过这种方式，可以实时对数据流进行平滑处理，减少噪声干扰，常见于嵌入式系统的传感器数据处理场景。*/
typedef struct {
  size_t size;   //number of values used for filtering
  size_t index;  //current value index
  size_t count;  //value count
  int sum;
  int *values;  //array to be filled with values
} ra_filter_t;

static ra_filter_t ra_filter;

/*这段代码是一个初始化函数，用于初始化一个 ra_filter_t 类型的结构体实例。我们来逐句分析：
代码解析
static ra_filter_t *：函数返回值为 ra_filter_t 类型的指针，且函数被声明为 static（静态函数）。
静态函数的作用域仅限于当前文件，其他文件无法调用（类似 “全局静态变量” 的作用域特性）。
函数名：ra_filter_init（推测是 “某个滤波器的初始化函数”）。
参数：
ra_filter_t *filter：指向待初始化的 ra_filter_t 结构体的指针（需外部提前分配内存）。
size_t sample_size：样本大小（可能用于初始化滤波器的参数，如缓冲区大小等）。
核心语句：
memset(filter, 0, sizeof(ra_filter_t));
memset 是 C 标准库函数，用于将一块内存区域的每个字节都设置为指定值（这里是 0）。
作用：将 filter 指向的 ra_filter_t 结构体所有成员初始化为 0（包括数值型变量置 0，指针置为 NULL 等），避免未初始化的垃圾值导致错误。
隐含信息
函数设计假设 filter 指针已指向一块有效的内存（可能是栈上分配，或通过 malloc 等动态分配），否则 memset 会操作非法内存导致崩溃。
static 修饰函数表明该初始化逻辑仅在当前文件内使用，不对外暴露，符合 “最小权限原则”。*/
static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size) {
  memset(filter, 0, sizeof(ra_filter_t));

  filter->values = (int *)malloc(sample_size * sizeof(int));
  if (!filter->values) {
    return NULL;
  }
  memset(filter->values, 0, sample_size * sizeof(int));

  filter->size = sample_size;
  return filter;
}

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
static int ra_filter_run(ra_filter_t *filter, int value) {
  if (!filter->values) {
    return value;
  }
  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = value;
  filter->sum += filter->values[filter->index];
  filter->index++;
  filter->index = filter->index % filter->size;
  if (filter->count < filter->size) {
    filter->count++;
  }
  return filter->sum / filter->count;
}
#endif

#if defined(LED_GPIO_NUM)
void enable_led(bool en) {  // Turn LED On or Off
/*三元运算符语法：条件 ? 表达式1 : 表达式2若条件为真（非 0），则返回表达式 1 的值；否则返回表达式 2 的值。
具体逻辑：
当 en 为 “真”（即 en != 0）时，duty 被赋值为 led_duty（可能是一个预设的占空比数值）。
当 en 为 “假”（即 en == 0）时，duty 被赋值为 0。*/
  int duty = en ? led_duty : 0;
  if (en && isStreaming && (led_duty > CONFIG_LED_MAX_INTENSITY)) {
    duty = CONFIG_LED_MAX_INTENSITY;
  }
  ledcWrite(LED_GPIO_NUM, duty);
  //ledc_set_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL, duty);
  //ledc_update_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL);
  log_i("Set LED intensity to %d", duty);
}
#endif

/*这行代码定义了一个静态函数 bmp_handler，从命名和参数来看，它很可能是一个用于处理 HTTP 请求中 BMP 格式相关内容的回调函数（常见于 ESP32 等嵌入式系统的 HTTP 服务器框架中）。
static 修饰符：表明该函数的作用域仅限于当前源文件，其他文件无法直接调用，符合模块化设计中 “内部接口隐藏” 的原则。
返回值 esp_err_t：这是 ESP-IDF（ESP32 官方开发框架）中常用的错误类型枚举，用于表示函数执行的结果状态（例如 ESP_OK 表示成功，ESP_FAIL 或其他特定值表示不同错误）。
函数名 bmp_handler：命名暗示其功能是 “处理 BMP 相关的请求”（BMP 是一种图像文件格式），可能用于响应客户端对 BMP 图片的请求（如返回图片数据）。
参数 httpd_req_t *req：
httpd_req_t 是 ESP-IDF 中 HTTP 服务器框架定义的结构体，用于封装一个 HTTP 请求的相关信息（如请求方法、URI、 headers、客户端连接等）。
指针 req 指向当前待处理的 HTTP 请求对象，函数通过该指针获取请求详情并生成响应。
函数的典型功能（推测）：
作为 HTTP 请求处理器，它的核心逻辑可能包括：
解析 req 中的请求信息（如确认客户端是否请求某个 BMP 图片）。
读取 BMP 图片数据（可能从 Flash、内存或文件系统中读取）。
通过 httpd_resp_send() 等函数向客户端发送 BMP 数据（设置正确的 Content-Type: image/bmp 等响应头）。
返回 esp_err_t 类型的结果（ESP_OK 表示响应成功，其他值表示处理失败）。
总结：
这是一个 ESP32 系统中用于处理 BMP 格式相关 HTTP 请求的静态回调函数，通过操作 httpd_req_t 结构体完成请求解析与响应，返回值用于标识处理状态。*/
static esp_err_t bmp_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  uint64_t fr_start = esp_timer_get_time();
  /*这行代码用于获取当前系统时间并存储到变量 fr_start 中，通常是嵌入式系统（尤其是 ESP32 平台）中常用的时间记录操作。
代码解析：
c
运行
uint64_t fr_start = esp_timer_get_time();
变量 fr_start：
类型为 uint64_t（64 位无符号整数），用于存储时间值，可表示较大的数值范围（避免溢出）。
命名中的 fr 可能是 “frame”（帧）的缩写，推测用于记录某一帧（如视频帧、数据帧）处理的起始时间。
函数 esp_timer_get_time()：
这是 ESP-IDF（ESP32 官方开发框架）提供的系统函数，用于获取当前系统运行时间。
返回值：以 微秒（μs） 为单位的时间戳（从系统启动开始计时，累计递增）。
例如：系统启动后 1 秒，返回值为 1000000（1e6 微秒）。
典型使用场景：
通常与后续的时间获取配合，计算某段代码的执行耗时，例如：
c
运行
// 记录开始时间
uint64_t fr_start = esp_timer_get_time();

// 执行某段需要计时的操作（如处理一帧数据）
process_frame();

// 计算耗时（微秒）
uint64_t fr_end = esp_timer_get_time();
uint64_t duration = fr_end - fr_start;
这样就可以得到 process_frame() 函数的执行时间（单位：微秒），常用于性能分析、帧率控制等场景。
特点：
精度高：以微秒为单位，适合测量短时间间隔（相比毫秒级更精确）。
单调性：时间戳随系统运行单调递增，不会因系统时间调整而回退。
总之，这行代码的核心作用是记录某一事件的起始时间（微秒级），为后续的时间差计算提供基准*/
#endif

/*esp_camera_fb_get() 函数：
这是 ESP-IDF 中摄像头驱动提供的函数，用于从摄像头获取一帧图像数据。
返回值 fb 是 esp_camera_fb_t* 类型的指针（fb 即 "frame buffer" 的缩写），指向存储图像数据的帧缓冲区结构体，包含图像的像素数据、宽度、高度、格式（如 JPEG、RGB 等）等信息。
若摄像头未就绪、硬件故障或资源不足，该函数会返回 NULL，表示获取失败。
失败处理逻辑：
if (!fb)：判断是否获取失败（fb 为 NULL）。
log_e("Camera capture failed")：通过 log_e（ESP-IDF 的错误日志宏）输出错误信息，便于调试时定位问题。
httpd_resp_send_500(req)：若该代码处于 HTTP 服务器的请求处理流程中（结合前面的 httpd_req_t *req 参数），则向客户端发送 HTTP 500 响应（"Internal Server Error"），告知客户端服务器处理失败。
return ESP_FAIL：函数返回 ESP_FAIL（ESP-IDF 定义的错误状态码），向上层调用者传递失败信息。
后续操作（隐含逻辑）：
若获取成功（fb 非 NULL），后续通常会：
处理图像数据（如压缩、分析、传输等），通过 fb->buf 访问像素数据，fb->len 获取数据长度，fb->width/fb->height 获取图像尺寸。
使用完毕后，必须调用 esp_camera_fb_return(fb) 释放帧缓冲区，避免资源泄漏（摄像头的帧缓冲区通常是有限的共享资源）。*/
  fb = esp_camera_fb_get();
  if (!fb) {
    log_e("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

/*httpd_resp_set_type(req, "image/x-windows-bmp")：
作用：设置 HTTP 响应的 Content-Type 头部，告知客户端返回数据的 MIME 类型（即数据格式）。
image/x-windows-bmp 是 BMP 图像格式的标准 MIME 类型，客户端（如浏览器）收到后会识别为图片并正确渲染。
若不设置，客户端可能无法识别数据格式（例如当作纯文本显示，导致乱码）。
httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.bmp")：
作用：设置 Content-Disposition 头部，控制客户端对数据的处理方式。
inline：表示数据应 “在线显示”（如浏览器直接显示图片，而非下载）。
filename=capture.bmp：指定默认文件名（若客户端选择下载，会默认使用该文件名保存）。
若改为 attachment; filename=xxx.bmp，则客户端会直接触发下载行为。
httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*")：
作用：设置跨域资源共享（CORS）头部，解决前端跨域请求的限制。
浏览器的同源策略会阻止不同域名的前端页面（如 http://a.com 的页面）向 http://b.com 的服务器发送请求，除非服务器返回允许跨域的头部。
"*" 表示允许任何域名的客户端访问，适合开发阶段或公开资源的场景（生产环境可能需要指定具体域名以提高安全性）。
结合上下文的意义
这些代码通常用于 HTTP 服务器的图片响应逻辑中（例如前面提到的 bmp_handler 函数）：
先通过摄像头获取图像（esp_camera_fb_get），转换为 BMP 格式。
再通过上述代码设置响应头，告知客户端 “这是一张 BMP 图片，直接显示，文件名叫 capture.bmp，允许跨域访问”。
最后通过 httpd_resp_send(req, fb->buf, fb->len) 发送图片数据，完成响应。
总结
这三行代码通过设置 HTTP 响应头，确保客户端能正确识别、显示 BMP 图片，并解决跨域访问问题，是 Web 服务中处理图片响应的标准操作，保证了客户端与服务器之间的数据交互规范。*/
  httpd_resp_set_type(req, "image/x-windows-bmp");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.bmp");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

/*char ts[32]：声明一个长度为 32 的字符数组 ts（ts 即 "timestamp" 的缩写），用于存储格式化后的时间戳字符串。32 的长度足以容纳最大可能的时间戳（秒数为 64 位整数，微秒为 6 位，加上分隔符 .，总长度远小于 32）。
snprintf 格式化时间戳：
作用：将图像采集的秒级时间和微秒级时间拼接为一个高精度时间戳字符串。
格式字符串 "%lld.%06ld"：
%lld：用于输出 long long 类型的 fb->timestamp.tv_sec（秒数，从某个时间起点开始的累计秒数，通常是 Unix 时间戳）。
%06ld：用于输出 long 类型的 fb->timestamp.tv_usec（微秒数，范围 0-999999），06 表示不足 6 位时用 0 补全（例如微秒数 123 会格式化为 000123）。
结果示例：若 tv_sec=1718000000、tv_usec=123456，则 ts 会被格式化为 "1718000000.123456"。
添加自定义响应头 X-Timestamp：
httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts) 向 HTTP 响应中添加一个自定义头部 X-Timestamp，其值为格式化后的时间戳字符串。
自定义头部通常以 X- 为前缀，用于传递额外的元数据（这里是图像的采集时间）。客户端收到响应后，可以通过读取该头部获取图像的精确生成时间（例如用于日志记录、同步处理等）。
结合上下文的意义
从变量 fb（帧缓冲区）可知，这段代码通常跟随在摄像头采集图像之后（fb = esp_camera_fb_get()），fb->timestamp 是摄像头驱动记录的图像采集时刻的时间戳（包含秒和微秒，精确到微秒级）。
通过将这个高精度时间戳放入响应头，客户端可以明确知道这张图像是何时采集的，对于需要时间同步的场景（如视频流时间校准、事件时序分析等）非常有用。*/
  char ts[32];
  snprintf(ts, 32, "%lld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
  httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);


/*这段代码的核心功能是将摄像头获取的原始图像帧（fb）转换为 BMP 格式，并处理转换失败的情况，是生成 BMP 图像响应的关键步骤（结合前文的 HTTP 响应逻辑）。
代码解析：
c
运行
// 定义指针和长度变量，用于接收转换后的BMP数据
uint8_t *buf = NULL;
size_t buf_len = 0;

// 调用frame2bmp函数，将摄像头帧数据转换为BMP格式
bool converted = frame2bmp(fb, &buf, &buf_len);

// 释放摄像头帧缓冲区（已完成转换，不再需要原始帧数据）
esp_camera_fb_return(fb);

// 检查转换是否成功
if (!converted) {
    log_e("BMP Conversion failed"); // 输出转换失败日志
    httpd_resp_send_500(req);      // 向客户端发送500错误响应
    return ESP_FAIL;               // 返回失败状态
}
各步骤的作用：
初始化变量：
uint8_t *buf = NULL：声明一个指针 buf，用于接收转换后的 BMP 图像数据（BMP 格式的字节流）。
size_t buf_len = 0：声明一个变量 buf_len，用于接收 BMP 数据的总长度（字节数）。
这两个变量通过指针传递给转换函数，用于 “输出” 转换结果。
图像格式转换（frame2bmp）：
函数 frame2bmp 是自定义或库函数，作用是将摄像头获取的原始帧数据（fb 指向的 esp_camera_fb_t 结构体，可能是 JPEG、RGB 等格式）转换为 BMP 格式。
参数说明：
fb：输入参数，指向原始图像帧数据（包含像素缓冲区、格式、尺寸等信息）。
&buf：输出参数，转换后的 BMP 数据会存储到 buf 指向的内存（函数内部可能通过 malloc 分配）。
&buf_len：输出参数，用于返回 BMP 数据的总长度（供后续发送数据时使用）。
返回值 converted 是布尔值：true 表示转换成功，false 表示失败（如格式不支持、内存不足等）。
释放原始帧缓冲区：
esp_camera_fb_return(fb)：将摄像头的原始帧缓冲区归还给系统。由于原始帧数据已用于转换，且 BMP 转换后的数据存储在 buf 中，因此需要及时释放 fb 占用的资源（摄像头的帧缓冲区是有限的共享资源，不释放会导致后续采集失败）。
转换失败处理：
if (!converted)：判断转换是否失败。
失败时：输出错误日志（log_e）、向 HTTP 客户端发送 500 错误响应（服务器内部错误）、返回 ESP_FAIL 告知上层调用者处理失败。
注意：若转换失败，buf 可能未分配内存（或需要手动释放，取决于 frame2bmp 的实现），需避免内存泄漏。
后续隐含操作（成功时）：
若转换成功（converted = true），后续通常会：
通过前文设置的 HTTP 响应头（Content-Type: image/x-windows-bmp 等），将 buf 中的 BMP 数据发送给客户端：httpd_resp_send(req, (const char*)buf, buf_len)。
发送完成后，释放 buf 指向的内存（如 free(buf)），避免内存泄漏。
总结：*/
  uint8_t *buf = NULL;
  size_t buf_len = 0;
  bool converted = frame2bmp(fb, &buf, &buf_len);
  esp_camera_fb_return(fb);
  if (!converted) {
    log_e("BMP Conversion failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  res = httpd_resp_send(req, (const char *)buf, buf_len);
  free(buf);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  uint64_t fr_end = esp_timer_get_time();
#endif
  log_i("BMP: %llums, %uB", (uint64_t)((fr_end - fr_start) / 1000), buf_len);
  return res;
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len) {
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index) {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
    return 0;
  }
  j->len += len;
  return len;
}

static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_start = esp_timer_get_time();
#endif

#if defined(LED_GPIO_NUM)
  enable_led(true);
  vTaskDelay(150 / portTICK_PERIOD_MS);  // The LED needs to be turned on ~150ms before the call to esp_camera_fb_get()
  fb = esp_camera_fb_get();              // or it won't be visible in the frame. A better way to do this is needed.
  enable_led(false);
#else
  fb = esp_camera_fb_get();
#endif

  if (!fb) {
    log_e("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char ts[32];
  snprintf(ts, 32, "%lld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
  httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  size_t fb_len = 0;
#endif
  if (fb->format == PIXFORMAT_JPEG) {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    fb_len = fb->len;
#endif
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  } else {
    jpg_chunking_t jchunk = {req, 0};
    res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
    httpd_resp_send_chunk(req, NULL, 0);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    fb_len = jchunk.len;
#endif
  }
  esp_camera_fb_return(fb);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_end = esp_timer_get_time();
#endif
  log_i("JPG: %uB %ums", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
  return res;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  struct timeval _timestamp;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[128];

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "60");

#if defined(LED_GPIO_NUM)
  isStreaming = true;
  enable_led(true);
#endif

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      log_e("Camera capture failed");
      res = ESP_FAIL;
    } else {
      _timestamp.tv_sec = fb->timestamp.tv_sec;
      _timestamp.tv_usec = fb->timestamp.tv_usec;
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          log_e("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      log_e("Send frame failed");
      break;
    }
    int64_t fr_end = esp_timer_get_time();

    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;

    frame_time /= 1000;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
#endif
    log_i(
      "MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps)", (uint32_t)(_jpg_buf_len), (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time, avg_frame_time,
      1000.0 / avg_frame_time
    );
  }

#if defined(LED_GPIO_NUM)
  isStreaming = false;
  enable_led(false);
#endif

  return res;
}

static esp_err_t parse_get(httpd_req_t *req, char **obuf) {
  char *buf = NULL;
  size_t buf_len = 0;

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      *obuf = buf;
      return ESP_OK;
    }
    free(buf);
  }
  httpd_resp_send_404(req);
  return ESP_FAIL;
}

static esp_err_t cmd_handler(httpd_req_t *req) {
  char *buf = NULL;
  char variable[32];
  char value[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) != ESP_OK || httpd_query_key_value(buf, "val", value, sizeof(value)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int val = atoi(value);
  log_i("%s = %d", variable, val);
  sensor_t *s = esp_camera_sensor_get();
  int res = 0;

  if (!strcmp(variable, "framesize")) {
    if (s->pixformat == PIXFORMAT_JPEG) {
      res = s->set_framesize(s, (framesize_t)val);
    }
  } else if (!strcmp(variable, "quality")) {
    res = s->set_quality(s, val);
  } else if (!strcmp(variable, "contrast")) {
    res = s->set_contrast(s, val);
  } else if (!strcmp(variable, "brightness")) {
    res = s->set_brightness(s, val);
  } else if (!strcmp(variable, "saturation")) {
    res = s->set_saturation(s, val);
  } else if (!strcmp(variable, "gainceiling")) {
    res = s->set_gainceiling(s, (gainceiling_t)val);
  } else if (!strcmp(variable, "colorbar")) {
    res = s->set_colorbar(s, val);
  } else if (!strcmp(variable, "awb")) {
    res = s->set_whitebal(s, val);
  } else if (!strcmp(variable, "agc")) {
    res = s->set_gain_ctrl(s, val);
  } else if (!strcmp(variable, "aec")) {
    res = s->set_exposure_ctrl(s, val);
  } else if (!strcmp(variable, "hmirror")) {
    res = s->set_hmirror(s, val);
  } else if (!strcmp(variable, "vflip")) {
    res = s->set_vflip(s, val);
  } else if (!strcmp(variable, "awb_gain")) {
    res = s->set_awb_gain(s, val);
  } else if (!strcmp(variable, "agc_gain")) {
    res = s->set_agc_gain(s, val);
  } else if (!strcmp(variable, "aec_value")) {
    res = s->set_aec_value(s, val);
  } else if (!strcmp(variable, "aec2")) {
    res = s->set_aec2(s, val);
  } else if (!strcmp(variable, "dcw")) {
    res = s->set_dcw(s, val);
  } else if (!strcmp(variable, "bpc")) {
    res = s->set_bpc(s, val);
  } else if (!strcmp(variable, "wpc")) {
    res = s->set_wpc(s, val);
  } else if (!strcmp(variable, "raw_gma")) {
    res = s->set_raw_gma(s, val);
  } else if (!strcmp(variable, "lenc")) {
    res = s->set_lenc(s, val);
  } else if (!strcmp(variable, "special_effect")) {
    res = s->set_special_effect(s, val);
  } else if (!strcmp(variable, "wb_mode")) {
    res = s->set_wb_mode(s, val);
  } else if (!strcmp(variable, "ae_level")) {
    res = s->set_ae_level(s, val);
  }
#if defined(LED_GPIO_NUM)
  else if (!strcmp(variable, "led_intensity")) {
    led_duty = val;
    if (isStreaming) {
      enable_led(true);
    }
  }
#endif
  else {
    log_i("Unknown command: %s", variable);
    res = -1;
  }

  if (res < 0) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static int print_reg(char *p, sensor_t *s, uint16_t reg, uint32_t mask) {
  return sprintf(p, "\"0x%x\":%u,", reg, s->get_reg(s, reg, mask));
}

static esp_err_t status_handler(httpd_req_t *req) {
  static char json_response[1024];

  sensor_t *s = esp_camera_sensor_get();
  char *p = json_response;
  *p++ = '{';

  if (s->id.PID == OV5640_PID || s->id.PID == OV3660_PID) {
    for (int reg = 0x3400; reg < 0x3406; reg += 2) {
      p += print_reg(p, s, reg, 0xFFF);  //12 bit
    }
    p += print_reg(p, s, 0x3406, 0xFF);

    p += print_reg(p, s, 0x3500, 0xFFFF0);  //16 bit
    p += print_reg(p, s, 0x3503, 0xFF);
    p += print_reg(p, s, 0x350a, 0x3FF);   //10 bit
    p += print_reg(p, s, 0x350c, 0xFFFF);  //16 bit

    for (int reg = 0x5480; reg <= 0x5490; reg++) {
      p += print_reg(p, s, reg, 0xFF);
    }

    for (int reg = 0x5380; reg <= 0x538b; reg++) {
      p += print_reg(p, s, reg, 0xFF);
    }

    for (int reg = 0x5580; reg < 0x558a; reg++) {
      p += print_reg(p, s, reg, 0xFF);
    }
    p += print_reg(p, s, 0x558a, 0x1FF);  //9 bit
  } else if (s->id.PID == OV2640_PID) {
    p += print_reg(p, s, 0xd3, 0xFF);
    p += print_reg(p, s, 0x111, 0xFF);
    p += print_reg(p, s, 0x132, 0xFF);
  }

  p += sprintf(p, "\"xclk\":%u,", s->xclk_freq_hz / 1000000);
  p += sprintf(p, "\"pixformat\":%u,", s->pixformat);
  p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
  p += sprintf(p, "\"quality\":%u,", s->status.quality);
  p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
  p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
  p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
  p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
  p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
  p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
  p += sprintf(p, "\"awb\":%u,", s->status.awb);
  p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
  p += sprintf(p, "\"aec\":%u,", s->status.aec);
  p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
  p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
  p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
  p += sprintf(p, "\"agc\":%u,", s->status.agc);
  p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
  p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
  p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
  p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
  p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
  p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
  p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
  p += sprintf(p, "\"vflip\":%u,", s->status.vflip);
  p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
  p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);
#if defined(LED_GPIO_NUM)
  p += sprintf(p, ",\"led_intensity\":%u", led_duty);
#else
  p += sprintf(p, ",\"led_intensity\":%d", -1);
#endif
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t xclk_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _xclk[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "xclk", _xclk, sizeof(_xclk)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int xclk = atoi(_xclk);
  log_i("Set XCLK: %d MHz", xclk);

  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_xclk(s, LEDC_TIMER_0, xclk);
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t reg_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _reg[32];
  char _mask[32];
  char _val[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "reg", _reg, sizeof(_reg)) != ESP_OK || httpd_query_key_value(buf, "mask", _mask, sizeof(_mask)) != ESP_OK
      || httpd_query_key_value(buf, "val", _val, sizeof(_val)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int reg = atoi(_reg);
  int mask = atoi(_mask);
  int val = atoi(_val);
  log_i("Set Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, val);

  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_reg(s, reg, mask, val);
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t greg_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _reg[32];
  char _mask[32];

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "reg", _reg, sizeof(_reg)) != ESP_OK || httpd_query_key_value(buf, "mask", _mask, sizeof(_mask)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int reg = atoi(_reg);
  int mask = atoi(_mask);
  sensor_t *s = esp_camera_sensor_get();
  int res = s->get_reg(s, reg, mask);
  if (res < 0) {
    return httpd_resp_send_500(req);
  }
  log_i("Get Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, res);

  char buffer[20];
  const char *val = itoa(res, buffer, 10);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, val, strlen(val));
}

static int parse_get_var(char *buf, const char *key, int def) {
  char _int[16];
  if (httpd_query_key_value(buf, key, _int, sizeof(_int)) != ESP_OK) {
    return def;
  }
  return atoi(_int);
}

static esp_err_t pll_handler(httpd_req_t *req) {
  char *buf = NULL;

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }

  int bypass = parse_get_var(buf, "bypass", 0);
  int mul = parse_get_var(buf, "mul", 0);
  int sys = parse_get_var(buf, "sys", 0);
  int root = parse_get_var(buf, "root", 0);
  int pre = parse_get_var(buf, "pre", 0);
  int seld5 = parse_get_var(buf, "seld5", 0);
  int pclken = parse_get_var(buf, "pclken", 0);
  int pclk = parse_get_var(buf, "pclk", 0);
  free(buf);

  log_i("Set Pll: bypass: %d, mul: %d, sys: %d, root: %d, pre: %d, seld5: %d, pclken: %d, pclk: %d", bypass, mul, sys, root, pre, seld5, pclken, pclk);
  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_pll(s, bypass, mul, sys, root, pre, seld5, pclken, pclk);
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t win_handler(httpd_req_t *req) {
  char *buf = NULL;

  if (parse_get(req, &buf) != ESP_OK) {
    return ESP_FAIL;
  }

  int startX = parse_get_var(buf, "sx", 0);
  int startY = parse_get_var(buf, "sy", 0);
  int endX = parse_get_var(buf, "ex", 0);
  int endY = parse_get_var(buf, "ey", 0);
  int offsetX = parse_get_var(buf, "offx", 0);
  int offsetY = parse_get_var(buf, "offy", 0);
  int totalX = parse_get_var(buf, "tx", 0);
  int totalY = parse_get_var(buf, "ty", 0);  // codespell:ignore totaly
  int outputX = parse_get_var(buf, "ox", 0);
  int outputY = parse_get_var(buf, "oy", 0);
  bool scale = parse_get_var(buf, "scale", 0) == 1;
  bool binning = parse_get_var(buf, "binning", 0) == 1;
  free(buf);

  log_i(
    "Set Window: Start: %d %d, End: %d %d, Offset: %d %d, Total: %d %d, Output: %d %d, Scale: %u, Binning: %u", startX, startY, endX, endY, offsetX, offsetY,
    totalX, totalY, outputX, outputY, scale, binning  // codespell:ignore totaly
  );
  sensor_t *s = esp_camera_sensor_get();
  int res = s->set_res_raw(s, startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning);  // codespell:ignore totaly
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    if (s->id.PID == OV3660_PID) {
      return httpd_resp_send(req, (const char *)index_ov3660_html_gz, index_ov3660_html_gz_len);
    } else if (s->id.PID == OV5640_PID) {
      return httpd_resp_send(req, (const char *)index_ov5640_html_gz, index_ov5640_html_gz_len);
    } else {
      return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
    }
  } else {
    log_e("Camera sensor not found");
    return httpd_resp_send_500(req);
  }
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 16;

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t status_uri = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t cmd_uri = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t capture_uri = {
    .uri = "/capture",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t bmp_uri = {
    .uri = "/bmp",
    .method = HTTP_GET,
    .handler = bmp_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t xclk_uri = {
    .uri = "/xclk",
    .method = HTTP_GET,
    .handler = xclk_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t reg_uri = {
    .uri = "/reg",
    .method = HTTP_GET,
    .handler = reg_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t greg_uri = {
    .uri = "/greg",
    .method = HTTP_GET,
    .handler = greg_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t pll_uri = {
    .uri = "/pll",
    .method = HTTP_GET,
    .handler = pll_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_uri_t win_uri = {
    .uri = "/resolution",
    .method = HTTP_GET,
    .handler = win_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  ra_filter_init(&ra_filter, 20);

  log_i("Starting web server on port: '%d'", config.server_port);
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &bmp_uri);

    httpd_register_uri_handler(camera_httpd, &xclk_uri);
    httpd_register_uri_handler(camera_httpd, &reg_uri);
    httpd_register_uri_handler(camera_httpd, &greg_uri);
    httpd_register_uri_handler(camera_httpd, &pll_uri);
    httpd_register_uri_handler(camera_httpd, &win_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  log_i("Starting stream server on port: '%d'", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setupLedFlash() {
#if defined(LED_GPIO_NUM)
  ledcAttach(LED_GPIO_NUM, 5000, 8);
#else
  log_i("LED flash is disabled -> LED_GPIO_NUM undefined");
#endif
}
