import urllib.request
import os
import sys

# 适用于Python 3.9的OpenCV和numpy版本（选择兼容的版本）
opencv_version = "4.5.5.64"
numpy_version = "1.21.6"

# 下载函数
def download_file(url, save_path):
    print(f"正在下载: {url}")
    urllib.request.urlretrieve(url, save_path)
    print(f"下载完成: {save_path}")

# 主函数
def main():
    # 创建临时目录
    temp_dir = "temp_wheels"
    if not os.path.exists(temp_dir):
        os.makedirs(temp_dir)
    
    try:
        # 获取当前Python版本信息
        py_version = f"cp{sys.version_info.major}{sys.version_info.minor}"
        print(f"使用Python版本: {py_version}")
        
        # 构建wheel文件名（Windows 64位）
        opencv_wheel = f"opencv_python-{opencv_version}-{py_version}-{py_version}-win_amd64.whl"
        numpy_wheel = f"numpy-{numpy_version}-{py_version}-{py_version}-win_amd64.whl"
        
        # 构建正确的下载URL（使用PyPI的wheel下载地址）
        # 注意：对于Windows预编译wheel，正确的路径格式是在wheels目录下
        opencv_url = f"https://download.lfd.uci.edu/pythonlibs/archived/{opencv_wheel}"
        numpy_url = f"https://download.lfd.uci.edu/pythonlibs/archived/{numpy_wheel}"
        
        # 下载wheel文件
        opencv_path = os.path.join(temp_dir, opencv_wheel)
        numpy_path = os.path.join(temp_dir, numpy_wheel)
        
        download_file(opencv_url, opencv_path)
        download_file(numpy_url, numpy_path)
        
        # 使用pip安装
        print("\n正在安装OpenCV和numpy...")
        os.system(f"pip install {opencv_path} {numpy_path}")
        
        print("\n安装完成！")
        
    except Exception as e:
        print(f"发生错误: {e}")
        print("尝试使用备用方法...")
        
        # 尝试直接从GitHub下载最新版本
        try:
            print("\n尝试从GitHub下载最新的OpenCV和numpy...")
            os.system(f"pip install --no-cache-dir --timeout 120 opencv-python numpy -i https://pypi.mirrors.ustc.edu.cn/simple/")
        except Exception as e2:
            print(f"备用方法也失败: {e2}")
    finally:
        # 清理临时文件
        print("\n清理临时文件...")
        if os.path.exists(temp_dir):
            for file in os.listdir(temp_dir):
                file_path = os.path.join(temp_dir, file)
                if os.path.isfile(file_path):
                    os.remove(file_path)
            os.rmdir(temp_dir)

if __name__ == "__main__":
    main()
