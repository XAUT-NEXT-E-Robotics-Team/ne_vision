# -*- coding: utf-8 -*-

import argparse
import os
import sys

import yaml

current_script_dir = os.path.dirname(os.path.abspath(__file__))
# 将该目录加入到 Python 的模块搜索路径中
if current_script_dir not in sys.path:
    sys.path.append(current_script_dir)

from scripts.nv_build import nv_build
from scripts.nv_run import nv_run
from scripts.nv_utils import *

# --- 1. 配置加载 ---


def load_config():
    """
    加载与此脚本位于同一目录的 settings.yaml 文件。

    这个函数演示了如何读取配置文件，即使文件是空的。
    这为未来通过文件进行配置提供了框架。

    返回:
        dict: 包含配置信息的字典。如果文件为空、不存在或解析失败，则返回 None。
    """
    config_path = ""
    try:
        # 获取当前脚本文件(build_tool.py)的绝对路径
        script_path = os.path.abspath(__file__)
        # 获取该脚本所在的目录
        script_dir = os.path.dirname(script_path)
        # 组合成配置文件的完整路径
        config_path = os.path.join(script_dir, "nv_config.yaml")

        with open(config_path, "r", encoding="utf-8") as f:
            # yaml.safe_load 在文件为空时会返回 None
            config_data = yaml.safe_load(f)
            print(f"信息：已成功读取配置文件 '{config_path}'。")
            if config_data is None:
                print("信息：配置文件为空，将使用脚本内置的默认设置。")
            return config_data

    except FileNotFoundError:
        print(f"警告：配置文件 'nv_config' 未在项目根目录中找到。")
        return None
    except Exception as e:
        print(f"错误：加载或解析 YAML 配置文件时出错: {e}")
        return None


# --- 2. 命令处理函数 (框架) ---


def handle_config(args, config):
    """处理 'config' 指令"""
    print("正在执行 'config' 指令...")
    if config:
        print("\n检测到配置文件内容:")
        print(yaml.dump(config, allow_unicode=True, default_flow_style=False))
    else:
        print("未从配置文件中加载任何内容。")
    print("这里可以添加显示或修改配置的逻辑。")
    print("-" * 20)


def handle_build(args, config):
    """处理 'build' 指令"""
    print("正在执行 'build' 指令...")
    nv_build(config)
    print("-" * 20)


def handle_run(args, config):
    """处理 'run' 指令"""
    target = args.run_target
    print(f"正在执行 'run' 指令，运行目标: '{target}'...")
    nv_run(config, config.get("exec_paths", {}), target)
    print("-" * 20)


# --- 3. 主程序入口 ---


def discover_run_targets(config, root_dir):
    """
    动态发现可执行文件并更新配置.

    Args:
        config (dict): 当前的配置字典.
        root_dir (str): 项目的根目录路径.

    Returns:
        dict: 更新后的配置字典.
    """
    if config is None:
        config = {}

    # 1. 从配置文件加载路径
    config_exec_paths = config.get("exec_paths", {})

    # 2. 从文件系统发现路径
    discovered_exec_paths = {}
    # 配置文件中的键是 install_dir, 用户提到了 install_path, 这里做兼容
    install_dir = config.get("install_dir", config.get("install_path", "install"))
    test_bin_dir = os.path.join(root_dir, install_dir, "test", "bin")

    if os.path.isdir(test_bin_dir):
        for filename in os.listdir(test_bin_dir):
            file_path = os.path.join(test_bin_dir, filename)
            # 检查是否是文件且有执行权限
            if os.path.isfile(file_path) and os.access(file_path, os.X_OK):
                discovered_exec_paths[filename] = file_path

    # 3. 合并两个字典，配置文件中的路径有更高优先级
    # 如果键冲突，以 config_exec_paths 中的值为准
    final_exec_paths = discovered_exec_paths.copy()
    final_exec_paths.update(config_exec_paths)

    config["exec_paths"] = final_exec_paths
    return config


def main():
    """
    主函数：定义框架、创建命令行解析器、解析参数并执行相应操作。
    """

    # 尝试加载外部配置，但脚本不依赖其内容来定义指令
    loaded_config = load_config()
    loaded_config = discover_run_targets(loaded_config, current_script_dir)

    VALID_RUN_TARGETS = tuple(loaded_config.get("exec_paths", {}).keys())

    VALID_RUN_TARGETS = tuple(
        ["ut"] + list(VALID_RUN_TARGETS)
    )  # 添加 'ut' 作为默认选项

    # --- 创建命令行解析器 ---
    parser = argparse.ArgumentParser(
        prog="build_tool.py",
        description="ne_vision的快捷功能脚本。",
        epilog="使用 '.nv.py <指令> --help' 来查看特定指令的详细帮助。",
    )

    # 创建子指令解析器
    subparsers = parser.add_subparsers(
        dest="command", required=True, help="可用的子指令"
    )

    # --- 'config' 指令 ---
    parser_config = subparsers.add_parser(
        "config", help="显示配置信息（演示读取配置文件）。"
    )
    parser_config.set_defaults(func=lambda args: handle_config(args, loaded_config))

    # --- 'build' 指令 ---
    parser_build = subparsers.add_parser("build", help="编译项目代码。")
    parser_build.set_defaults(func=lambda args: handle_build(args, loaded_config))

    # --- 'run' 指令 ---
    parser_run = subparsers.add_parser("run", help="运行一个指定的对象。")
    parser_run.add_argument(
        "run_target",
        metavar="运行对象",
        choices=VALID_RUN_TARGETS,
        help=f"选择一个要运行的对象: {', '.join(VALID_RUN_TARGETS)}",
    )
    parser_run.set_defaults(func=lambda args: handle_run(args, loaded_config))

    # 解析命令行参数
    # 如果没有提供任何参数，argparse 默认会显示错误。为了更友好，可以让他显示帮助信息
    if len(sys.argv) == 1:
        parser.print_help(sys.stderr)
        sys.exit(1)

    args = parser.parse_args()

    # 根据选择的子指令，调用对应的处理函数
    if hasattr(args, "func"):
        args.func(args)


if __name__ == "__main__":
    main()
