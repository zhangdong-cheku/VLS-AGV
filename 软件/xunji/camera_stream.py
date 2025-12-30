import cv2
import time
import urllib.request
import numpy as np
import asyncio
import threading
import queue
from bleak import BleakClient, BleakScanner

# 创建全局事件循环
loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

# BLE相关配置（从ESP32代码中获取）
BLE_DEVICE_NAME = "ESP32_Stepper_BLE"
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
RX_CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"  # 发送数据到ESP32
TX_CHARACTERISTIC_UUID = "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"  # 从ESP32接收数据


class BLEController:
    """BLE控制器类，用于与ESP32进行蓝牙通信"""
    
    def __init__(self):
        self.client = None
        self.is_connected = False
        self.device_address = None
        self.rx_characteristic = None
        self.tx_characteristic = None
    
    async def find_device(self):
        """查找ESP32 BLE设备"""
        print("正在扫描BLE设备...")
        
        # 扫描BLE设备
        devices = await BleakScanner.discover()
        
        for device in devices:
            if device.name == BLE_DEVICE_NAME:
                self.device_address = device.address
                print(f"找到设备: {BLE_DEVICE_NAME}, 地址: {self.device_address}")
                return True
        
        print(f"未找到设备: {BLE_DEVICE_NAME}")
        return False
    
    async def connect(self):
        """连接到ESP32 BLE设备"""
        if not self.device_address:
            if not await self.find_device():
                return False
        
        try:
            print(f"正在连接到 {self.device_address}...")
            self.client = BleakClient(self.device_address)
            await self.client.connect()
            self.is_connected = True
            print(f"成功连接到 {BLE_DEVICE_NAME}")
            
            # 查找服务和特征
            await self._find_characteristics()
            return True
        except Exception as e:
            print(f"连接失败: {str(e)}")
            self.is_connected = False
            return False
    
    async def _find_characteristics(self):
        """查找BLE服务和特征"""
        if not self.client or not self.is_connected:
            return False
        
        try:
            # 获取所有服务
            services = self.client.services
            
            # 查找目标服务
            target_service = None
            for service in services:
                if service.uuid == SERVICE_UUID:
                    target_service = service
                    break
            
            if not target_service:
                print(f"未找到服务: {SERVICE_UUID}")
                return False
            
            # 查找RX和TX特征
            for char in target_service.characteristics:
                if char.uuid == RX_CHARACTERISTIC_UUID:
                    self.rx_characteristic = char
                    print(f"找到RX特征: {RX_CHARACTERISTIC_UUID}")
                elif char.uuid == TX_CHARACTERISTIC_UUID:
                    self.tx_characteristic = char
                    print(f"找到TX特征: {TX_CHARACTERISTIC_UUID}")
            
            if not self.rx_characteristic or not self.tx_characteristic:
                print("未找到所需的特征")
                return False
            
            return True
        except Exception as e:
            print(f"查找特征失败: {str(e)}")
            return False
    
    async def send_command(self, command):
        """发送命令到ESP32"""
        if not self.client or not self.is_connected or not self.rx_characteristic:
            return False
        
        try:
            # 在命令末尾添加换行符，使ESP32能正确识别完整命令
            command_with_terminator = command + '\n'
            
            # 将命令转换为字节
            command_bytes = command_with_terminator.encode('utf-8')
            
            # 发送命令
            await self.client.write_gatt_char(self.rx_characteristic, command_bytes)
            return True
        except Exception as e:
            print(f"发送命令失败: {str(e)}")
            return False
    
    async def disconnect(self):
        """断开与ESP32的连接"""
        if self.client and self.is_connected:
            try:
                await self.client.disconnect()
                print(f"已断开与 {BLE_DEVICE_NAME} 的连接")
            except Exception as e:
                print(f"断开连接失败: {str(e)}")
            finally:
                self.is_connected = False
                self.client = None
                self.rx_characteristic = None
                self.tx_characteristic = None


# 创建命令队列，用于主线程向BLE线程发送命令
command_queue = queue.Queue(maxsize=5)  # 最大队列大小为5，避免内存溢出

# BLE通信线程类
class BLEThread(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.ble_controller = BLEController()
        self.running = False
        
    def run(self):
        self.running = True
        
        # 初始化BLE连接
        try:
            loop.run_until_complete(self.ble_controller.connect())
        except Exception as e:
            print(f"BLE连接初始化失败: {str(e)}")
            return
        
        # 处理命令队列
        while self.running:
            try:
                # 从队列获取命令，最多等待0.1秒
                command = command_queue.get(timeout=0.1)
                
                if command is None:
                    # None命令表示退出
                    break
                    
                # 发送命令
                loop.run_until_complete(self.ble_controller.send_command(command))
                command_queue.task_done()
            except queue.Empty:
                # 队列为空，继续循环
                continue
            except Exception as e:
                print(f"BLE线程错误: {str(e)}")
                break
        
        # 清理资源
        if self.ble_controller.is_connected:
            try:
                loop.run_until_complete(self.ble_controller.disconnect())
            except Exception as e:
                print(f"BLE断开连接失败: {str(e)}")
    
    def stop(self):
        self.running = False
        # 向队列发送None命令以确保线程退出
        if not command_queue.full():
            command_queue.put(None)

# 指令发送控制
last_command_time = 0  # 上次发送指令的时间（秒）
last_sent_command = None  # 上次发送的指令内容

# 创建BLE线程实例
ble_thread = BLEThread()

# 基础URL设置
BASE_URL = "http://192.168.0.168:81"

# ESP32相机常见的视频流路径列表
STREAM_PATHS = [
    "/mjpeg/1",    # 常见的MJPEG流路径
    "/stream",     # 常见的流路径
    "/mjpegfeed",  # 另一种常见的MJPEG流路径
    "/video",      # 视频流路径
    "/capture",    # 捕获路径（可能需要连续请求）
    "/"
]  # 根路径作为最后的尝试

# HSV颜色范围设置（红色）
# 红色在HSV中分为两个区间
lower_red1 = np.array([0, 100, 100])    # 低区间红色
upper_red1 = np.array([10, 255, 255])   # 低区间红色上限
lower_red2 = np.array([160, 100, 100])  # 高区间红色
upper_red2 = np.array([180, 255, 255])  # 高区间红色上限

# 形态学处理的核 - 优化：减小核大小以降低计算量
kernel = np.ones((3, 3), np.uint8)


def offset_to_motor_command(offset):
    """
    将偏移量转换为ESP32电机控制命令
    
    参数:
        offset: 红色地标相对于图像中心的偏移量（像素）
    
    返回:
        command: ESP32可执行的电机控制命令
    """
    # 根据用户要求的控制策略
    if -100 <= offset <= 100:
        return "SPEEDL100,R100"  # 中心区域，双电机100%速度
    elif 100 < offset <= 150:
        return "SPEEDL100,R60"   # 右偏移，左电机100%、右电机60%
    elif offset > 150:
        return "SPEEDL100,R40"   # 右偏移，左电机100%、右电机40%
    elif  -150 < offset < -100:
        return "SPEEDL60,R100"   # 左偏移，左电机60%、右电机100%
    elif offset < -150:
         return "SPEEDL40,R100"   # 左偏移，左电机40%、右电机100%

        
        # 边缘区间（10<offset≤12 或 -12≤offset<-10）
        # 可以根据实际情况调整，这里暂时使用与相邻区间相同的命令
     

def extract_red_landmark(frame):
    """
    提取红色地标
    步骤0：裁剪图像到最上面区域
    步骤1：转换颜色空间
    步骤2：定义红色HSV范围并生成掩码
    步骤3：形态学处理去除噪声
    """
    # 步骤0：裁剪图像到最上面区域（上半部分）
    height, width = frame.shape[:2]
    crop_top = int(height * 0.3)  # 只保留图像顶部30%的区域
    frame = frame[0:crop_top, 0:width]
    
    # 步骤1：将BGR转换为HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 步骤2：生成红色掩码（两个区间合并）
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)
    
    # 步骤3：形态学处理 - 先腐蚀再膨胀，去除噪声（优化：减少迭代次数）
    red_mask = cv2.erode(red_mask, kernel, iterations=1)  # 保持腐蚀1次
    red_mask = cv2.dilate(red_mask, kernel, iterations=1)  # 膨胀从2次减少到1次
    
    return red_mask

def calculate_offset_and_center(frame, mask):
    """
    计算红色地标的中心与偏移量
    步骤1：提取轮廓
    步骤2：筛选有效轮廓
    步骤3：计算地标中心
    步骤4：计算偏移量
    """
    # 步骤1：提取轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 步骤2：筛选有效轮廓（找面积最大的轮廓）
    if not contours:
        return None, None
    
    # 找出最大的轮廓
    largest_contour = max(contours, key=cv2.contourArea)
    
    # 过滤过小的轮廓（排除噪声）
    contour_area = cv2.contourArea(largest_contour)
    if contour_area < 100:  # 面积阈值，可根据实际情况调整
        return None, None
    
    # 步骤3：计算地标中心（质心）
    M = cv2.moments(largest_contour)
    if M['m00'] == 0:
        return None, None
    
    cx = int(M['m10'] / M['m00'])  # 中心x坐标
    cy = int(M['m01'] / M['m00'])  # 中心y坐标
    landmark_center = (cx, cy)
    
    # 步骤4：计算偏移量
    frame_center_x = frame.shape[1] // 2  # 图像水平中心
    offset = cx - frame_center_x          # 偏移量
    
    return landmark_center, offset

def visualize_results(frame, landmark_center, offset, mask):
    """
    可视化调试：标记地标中心、图像中心线，显示偏移量
    """
    # 创建一个副本用于显示
    display_frame = frame.copy()
    
    # 绘制图像中心线（蓝色竖线）
    frame_center_x = frame.shape[1] // 2
    cv2.line(display_frame, (frame_center_x, 0), (frame_center_x, frame.shape[0]), (255, 0, 0), 2)
    
    # 如果找到了地标
    if landmark_center is not None:
        # 绘制地标中心（红色圆点）
        cv2.circle(display_frame, landmark_center, 5, (0, 0, 255), -1)
        
        # 显示偏移量文本
        offset_text = f"偏移量: {offset}"
        cv2.putText(display_frame, offset_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 绘制从图像中心到地标中心的连线
        cv2.line(display_frame, (frame_center_x, landmark_center[1]), 
                landmark_center, (0, 255, 255), 2)
    else:
        # 未找到地标时显示提示
        cv2.putText(display_frame, "未检测到红色地标", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    # 创建掩码的彩色版本以便显示
    mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    # 水平拼接显示原始图像、处理后的图像和掩码
    combined = cv2.hconcat([display_frame, mask_color])
    
    # 调整窗口大小（如果图像太大）- 优化：降低最大宽度以提高流畅度
    max_width = 1000  # 从1200降低到1000，减少显示窗口大小
    if combined.shape[1] > max_width:
        scale = max_width / combined.shape[1]
        new_height = int(combined.shape[0] * scale)
        combined = cv2.resize(combined, (max_width, new_height))
    
    return combined

# 方法1：使用urllib直接获取MJPEG流并解码
def get_frame_from_stream(stream_url):
    try:
        stream = urllib.request.urlopen(stream_url)
        bytes_data = bytes()
        
        while True:
            # 读取数据
            chunk = stream.read(1024)
            if not chunk:
                break
            bytes_data += chunk
            
            a = bytes_data.find(b'\xff\xd8')  # JPEG起始标记
            b = bytes_data.find(b'\xff\xd9')  # JPEG结束标记
            
            if a != -1 and b != -1:
                jpg = bytes_data[a:b+2]  # 提取JPEG图像
                bytes_data = bytes_data[b+2:]  # 保留剩余数据
                
                # 解码JPEG图像
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                return True, frame
    except Exception as e:
        return False, str(e)

# 方法2：尝试使用OpenCV直接打开流
def try_opencv_direct(url):
    try:
        cap = cv2.VideoCapture(url)
        
        if not cap.isOpened():
            cap.release()
            return None
        
        # 优化视频流参数设置
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 设置缓冲区大小为1帧，减少延迟
        cap.set(cv2.CAP_PROP_FPS, 30)        # 设置期望帧率
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 1) # 确保输出RGB格式
        
        return cap
    except Exception as e:
        return None

def process_frame(frame):
    """
    处理单帧图像的完整流程
    """
    try:
        # 优化1：降低图像分辨率以提高处理速度
        scale = 0.4  # 增加缩放因子，进一步降低分辨率（从0.5→0.4）
        small_frame = cv2.resize(frame, (0, 0), fx=scale, fy=scale)
        
        # 步骤1：提取红色地标
        red_mask = extract_red_landmark(small_frame)
        
        # 步骤2：计算红色地标的中心与偏移量
        landmark_center, offset = calculate_offset_and_center(small_frame, red_mask)
        
        # 还原地标中心坐标到原始分辨率
        if landmark_center is not None:
            landmark_center = (int(landmark_center[0] / scale), int(landmark_center[1] / scale))
            if offset is not None:
                offset = int(offset / scale)  # 还原偏移量
        
        # 步骤3：生成可视化结果
        combined_frame = visualize_results(frame, landmark_center, offset, cv2.resize(red_mask, (frame.shape[1], frame.shape[0])))
        
        # 生成控制指令（如果需要）
        control_command = None
        motor_command = None
        
        if offset is not None:
            # 生成电机控制命令（用于发送到ESP32）
            motor_command = offset_to_motor_command(offset)
            
            # 如果电机命令不为空，将其放入队列（只有当命令发生变化时才发送）
            if motor_command:
                global last_command_time, last_sent_command
                current_time = time.time()
                
                # 检查是否是新命令
                if motor_command != last_sent_command:
                    # 将命令放入队列，不阻塞主线程
                    try:
                        # 非阻塞方式放入队列，如果队列已满则忽略
                        command_queue.put_nowait(motor_command)
                        last_command_time = current_time  # 更新最后发送时间
                        last_sent_command = motor_command  # 更新最后发送的命令
                        # 在这里打印命令，只在发送时打印一次
                        print(f"电机控制命令: {motor_command}")
                    except queue.Full:
                        # 队列已满，忽略该命令（避免主线程阻塞）
                        pass
        
        return combined_frame, control_command, offset, motor_command
    except Exception as e:
        print(f"图像处理失败: {str(e)}")
        # 返回原始帧作为备选
        return frame, None, None, None

def main():
    # 启动BLE线程
    print("启动BLE通信线程...")
    ble_thread.start()
    
    # 尝试每个可能的流路径
    for path in STREAM_PATHS:
        stream_url = BASE_URL + path
        print(f"\n尝试路径: {stream_url}")
        
        # 首先尝试使用OpenCV直接打开
        cap = try_opencv_direct(stream_url)
        if cap:
            # 如果OpenCV成功打开，使用常规方法读取视频
            print("视频流已成功打开。按'q'键退出，按'h'键显示帮助。")
            
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("无法获取视频帧。尝试重新连接...")
                    break
                
                # 处理帧
                combined_frame, control_command, offset, motor_command = process_frame(frame)
                
                # 显示处理结果
                cv2.imshow('ESP32 Camera 红色地标识别', combined_frame)
                
                # 电机控制命令已在process_frame函数中打印
                
                # 等待1ms并检查按键
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q'):
                    print("用户退出")
                    cap.release()
                    cv2.destroyAllWindows()
                    return
                elif key & 0xFF == ord('h'):
                    print("\n帮助信息:")
                    print("- q: 退出程序")
                    print("- h: 显示此帮助")
                    print("- c: 进入命令输入模式")
                    print("程序将显示原始图像和红色掩码，并标记地标的中心位置")
                    print("蓝色竖线表示图像中心线，红色圆点表示地标中心")
                    print("偏移量显示在左上角，负值表示地标在左侧，正值表示在右侧")
                elif key & 0xFF == ord('c'):
                    print("\n=== 命令输入模式 ===")
                    print("输入ESP32控制命令（输入'exit'退出此模式）")
                    print("例如: SPEEDL100,R100 (左右电机100%速度)")
                    print("      SPEEDL90,R100 (左电机90%，右电机100%)")
                    print("      S (停止所有电机)")
                    print("=====================")
                    
                    while True:
                        # 暂停视频流处理
                        command = input("命令: ").strip()
                        
                        if command.lower() == 'exit':
                            print("退出命令输入模式")
                            break
                        
                        if command:
                            try:
                                # 将命令放入队列，不阻塞主线程
                                command_queue.put_nowait(command)
                            except queue.Full:
                                print("命令队列已满，无法发送命令")
                            except Exception as e:
                                print(f"发送命令失败: {str(e)}")
                
                # 确保窗口不会被关闭
                if not cv2.getWindowProperty('ESP32 Camera 红色地标识别', cv2.WND_PROP_VISIBLE):
                    print("可视化窗口已被关闭，程序退出")
                    cap.release()
                    cv2.destroyAllWindows()
                    return
            
            cap.release()
        
        # 如果OpenCV直接打开失败，尝试使用urllib方式
        print(f"尝试使用urllib获取MJPEG流: {stream_url}")
        print("视频流打开中... (按'q'键切换到下一个路径或退出，按'h'键显示帮助)")
        
        while True:
            # 获取一帧图像
            success, frame = get_frame_from_stream(stream_url)
            
            if not success:
                print(f"  使用urllib获取失败: {frame}")
                break
            
            # 处理帧
            combined_frame, control_command, offset, motor_command = process_frame(frame)
            
            # 显示处理结果
            cv2.imshow('ESP32 Camera 红色地标识别', combined_frame)
            
            # 电机控制命令已在process_frame函数中打印
            
            # 等待1ms并检查按键
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                print("用户退出")
                cv2.destroyAllWindows()
                return
            elif key & 0xFF == ord('n'):  # 按'n'键尝试下一个路径
                print("切换到下一个路径...")
                break
            elif key & 0xFF == ord('h'):
                print("\n帮助信息:")
                print("- q: 退出程序")
                print("- n: 切换到下一个流路径")
                print("- h: 显示此帮助")
                print("- c: 进入命令输入模式")
                print("程序将显示原始图像和红色掩码，并标记地标的中心位置")
                print("蓝色竖线表示图像中心线，红色圆点表示地标中心")
                print("偏移量显示在左上角，负值表示地标在左侧，正值表示在右侧")
            elif key & 0xFF == ord('c'):
                print("\n=== 命令输入模式 ===")
                print("输入ESP32控制命令（输入'exit'退出此模式）")
                print("例如: SPEEDL100,R100 (左右电机100%速度)")
                print("      SPEEDL90,R100 (左电机90%，右电机100%)")
                print("      S (停止所有电机)")
                print("=====================")
                
                while True:
                    # 暂停视频流处理
                    command = input("命令: ").strip()
                    
                    if command.lower() == 'exit':
                        print("退出命令输入模式")
                        break
                    
                    if command:
                        try:
                            # 将命令放入队列，不阻塞主线程
                            command_queue.put_nowait(command)
                        except queue.Full:
                            print("命令队列已满，无法发送命令")
                        except Exception as e:
                            print(f"发送命令失败: {str(e)}")
            
            # 确保窗口不会被关闭
            if not cv2.getWindowProperty('ESP32 Camera 红色地标识别', cv2.WND_PROP_VISIBLE):
                print("可视化窗口已被关闭，切换到下一个路径")
                break
    
    # 如果所有路径都失败
    print("\n所有尝试的路径都无法连接到视频流。")
    print("请检查:")
    print("1. ESP32相机是否正常运行")
    print("2. 网络连接是否正常")
    print("3. IP地址和端口是否正确")
    print("4. 相机的流路径设置是否与程序中的不同")
    
    # 清理窗口
    cv2.destroyAllWindows()

if __name__ == "__main__":
    print("ESP32相机红色地标识别程序")
    print("目标：识别地面红色地标并计算偏移量")
    print("按'q'键退出，按'h'键显示帮助")
    print("\n注意：如果识别效果不佳，可以调整代码中的HSV参数")
    
    try:
        main()
    finally:
        # 程序结束时，停止BLE线程
        if ble_thread.is_alive():
            print("\n正在停止BLE通信线程...")
            try:
                ble_thread.stop()
                ble_thread.join(timeout=2.0)  # 等待线程结束，最多2秒
            except Exception as e:
                print(f"停止BLE线程失败: {str(e)}")