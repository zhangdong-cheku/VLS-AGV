import urllib.request
import os
import subprocess

# 下载函数（使用HTTP而非HTTPS，避免SSL问题）
def download_wheel(package_name, version, py_version, wheel_filename):
    print(f"正在下载 {package_name} {version}...")
    
    # 使用清华大学的HTTP镜像
    url = f"http://mirrors.tuna.tsinghua.edu.cn/pypi/packages/py3/{package_name[0]}/{package_name}/{wheel_filename}"
    
    try:
        # 设置请求头
        headers = {
            'User-Agent': 'Python-urllib/3.9'
        }
        
        req = urllib.request.Request(url, headers=headers)
        with urllib.request.urlopen(req) as response:
            data = response.read()
            
        with open(wheel_filename, 'wb') as f:
            f.write(data)
            
        print(f"下载完成: {wheel_filename}")
        return True
        
    except Exception as e:
        print(f"下载失败: {e}")
        print(f"尝试使用备用URL...")
        
        # 尝试另一个镜像
        url = f"http://pypi.doubanio.com/packages/py3/{package_name[0]}/{package_name}/{wheel_filename}"
        try:
            req = urllib.request.Request(url, headers=headers)
            with urllib.request.urlopen(req) as response:
                data = response.read()
                
            with open(wheel_filename, 'wb') as f:
                f.write(data)
                
            print(f"下载完成: {wheel_filename}")
            return True
            
        except Exception as e2:
            print(f"备用URL也失败: {e2}")
            return False

# 主函数
def main():
    # Python版本信息
    py_version = "cp39"
    
    # 选择兼容的版本
    opencv_version = "4.5.5.64"
    numpy_version = "1.21.6"
    
    # 构建wheel文件名
    opencv_wheel = f"opencv_python-{opencv_version}-{py_version}-{py_version}-win_amd64.whl"
    numpy_wheel = f"numpy-{numpy_version}-{py_version}-{py_version}-win_amd64.whl"
    
    # 下载wheel文件
    opencv_ok = download_wheel("opencv-python", opencv_version, py_version, opencv_wheel)
    numpy_ok = download_wheel("numpy", numpy_version, py_version, numpy_wheel)
    
    if not opencv_ok or not numpy_ok:
        print("\n下载失败，尝试直接安装...")
        # 使用pip直接安装，禁用SSL验证
        try:
            subprocess.run([
                "C:\Users\cnm\AppData\Local\Programs\Python\Python39\Scripts\pip.exe",
                "install",
                "--no-cache-dir",
                "--timeout", "120",
                "--trusted-host", "pypi.doubanio.com",
                "--index-url", "http://pypi.doubanio.com/simple/",
                "opencv-python",
                "numpy"
            ], check=True)
            print("\n安装成功！")
        except subprocess.CalledProcessError:
            print("\n安装失败，请尝试手动下载并安装wheel文件。")
            print("\n手动安装步骤：")
            print("1. 访问：https://www.lfd.uci.edu/~gohlke/pythonlibs/")
            print("2. 下载适用于Python 3.9的OpenCV和numpy wheel文件")
            print("3. 将文件保存到当前目录")
            print("4. 运行：pip install <opencv_wheel_file> <numpy_wheel_file>")
    else:
        # 安装本地wheel文件
        print("\n正在安装本地wheel文件...")
        try:
            subprocess.run([
                "C:\Users\cnm\AppData\Local\Programs\Python\Python39\Scripts\pip.exe",
                "install",
                opencv_wheel,
                numpy_wheel
            ], check=True)
            print("\n安装成功！")
            
            # 清理wheel文件
            os.remove(opencv_wheel)
            os.remove(numpy_wheel)
            print("\n已清理临时文件。")
        except subprocess.CalledProcessError:
            print("\n安装失败，请手动安装。")

if __name__ == "__main__":
    main()
