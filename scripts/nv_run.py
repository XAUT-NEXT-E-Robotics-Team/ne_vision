#!/usr/bin/env python3
import os
import sys

# ================= 导入 nv_utils =================
# 确保能找到同目录下的 nv_utils.py
current_script_dir = os.path.dirname(os.path.abspath(__file__))
if current_script_dir not in sys.path:
    sys.path.append(current_script_dir)
try:
    import nv_utils as utils
except ImportError:
    print("\033[1;31m[Fatal] Cannot find 'nv_utils.py'.\033[0m")
    sys.exit(1)

# ================= 主运行函数 =================


def nv_run(config: dict, targets_map: dict, target_name: str):
    """
    运行指定的构建目标或单元测试。

    Args:
        config (dict): 构建配置字典 (需包含 'build_dir')
        targets_map (dict): 运行对象映射 { '名称': '相对构建目录的可执行文件路径' }
        target_name (str): 要运行的目标名称 (如果是 'ut' 则运行测试)
    """

    # 1. 获取构建目录
    build_dir = config.get("build_dir", "build")

    # 2. 基础检查
    if not os.path.exists(build_dir):
        utils.log_error(f"Build directory not found: {build_dir}. Please build first.")
        return

    utils.log_process(f"Preparing to run target: [{target_name}]")

    # ================= 场景 A: 运行单元测试 (ut) =================
    if target_name == "ut":
        utils.log_info("Mode", "Unit Tests (CTest)")
        utils.log_process("Executing CTest...")

        # ctest 命令: --output-on-failure 会在测试失败时打印详细日志
        ctest_cmd = ["ctest", "--output-on-failure"]

        # 注意：CTest 必须在 build 目录下运行，或者指定 -C <Config>
        try:
            utils.run_command(ctest_cmd, cwd=build_dir)
            utils.log_process("All unit tests passed.")
        except SystemExit:
            utils.log_error("Unit tests failed.", exit_code=1)
        return

    # ================= 场景 B: 运行指定可执行文件 =================
    if target_name in targets_map:
        # 获取相对路径
        rel_path = targets_map[target_name]

        # 拼接绝对路径 (构建目录 + 相对路径)
        exe_path = os.path.join(os.path.abspath(build_dir), rel_path)

        # 针对 Windows 的兼容处理 (如果没写 .exe)
        if sys.platform == "win32" and not exe_path.lower().endswith(".exe"):
            if os.path.exists(exe_path + ".exe"):
                exe_path += ".exe"

        utils.log_info("Target", target_name)
        utils.log_info("Executable", exe_path)

        if not os.path.exists(exe_path):
            utils.log_error(f"Executable file not found: {exe_path}")
            utils.log_warning(f"Check your 'targets_map' or build status.")
            sys.exit(1)

        utils.log_process(f"Launching {target_name}...")
        print("-" * 60)

        # 运行程序
        # cwd=build_dir 通常是比较好的默认行为，方便程序找到同级资源
        # 如果需要从项目根目录运行，可以改为 cwd=config.get("project_dir")
        try:
            utils.run_command([exe_path], cwd=build_dir)
        except SystemExit:
            # 这里的 SystemExit 是 run_command 抛出的，代表程序返回非0
            utils.log_error(f"Target '{target_name}' exited with error.")

        print("-" * 60)
        utils.log_process(f"Target '{target_name}' finished.")

    else:
        # ================= 场景 C: 未知目标 =================
        utils.log_error(f"Unknown target name: '{target_name}'")
        utils.log_warning(f"Available targets: {list(targets_map.keys())} + ['ut']")
        sys.exit(1)
