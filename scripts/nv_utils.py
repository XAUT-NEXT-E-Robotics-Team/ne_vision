# nv_utils.py
import os
import shutil
import subprocess
import sys
from pathlib import Path


def log_process(msg):
    """打印构建过程信息 [PROCESS]"""
    print(f"\033[1;32m[PROCESS] {msg}\033[0m")


def log_info(key, value):
    """打印配置信息 [INFO]"""
    print(f"\033[1;34m[INFO]    {key:<20}: {value}\033[0m")


def log_warning(msg):
    """打印警告信息 [WARNING]"""
    print(f"\033[1;33m[WARNING] {msg}\033[0m")


def log_error(msg, exit_code=1):
    """打印错误信息 [ERROR] 并可选退出"""
    print(f"\033[1;31m[ERROR]   {msg}\033[0m", file=sys.stderr)
    if exit_code is not None:
        sys.exit(exit_code)


def check_tool(tool_name):
    """检查系统路径中是否存在指定工具"""
    return shutil.which(tool_name) is not None


def run_command(command, cwd=None, env=None):
    """执行 Shell 命令，失败则直接退出"""
    cmd_str = " ".join(command)
    log_process(f"Exec: {cmd_str}")
    try:
        current_env = os.environ.copy()
        if env:
            current_env.update(env)

        subprocess.run(command, cwd=cwd, env=current_env, check=True)
    except subprocess.CalledProcessError:
        log_error(f"Command execution failed: {cmd_str}")
    except FileNotFoundError:
        log_error(f"Command not found: {command[0]}")


def get_files_dict_pathlib(folder_path):
    # 将输入转换为 Path 对象
    p = Path(folder_path)

    # 1. 检查文件夹是否存在，且确实是一个目录
    if not p.exists() or not p.is_dir():
        return {}

    # 2. 遍历文件夹，筛选出文件（排除子文件夹），并构建字典
    # f.name 是文件名，str(f.resolve()) 是绝对路径
    files_dict = {f.name: str(f.resolve()) for f in p.iterdir() if f.is_file()}

    return files_dict
