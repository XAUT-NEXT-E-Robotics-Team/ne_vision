# build.py
import os
import shutil
import sys

# 获取当前脚本文件的绝对路径目录
current_script_dir = os.path.dirname(os.path.abspath(__file__))
# 将该目录加入到 Python 的模块搜索路径中
if current_script_dir not in sys.path:
    sys.path.append(current_script_dir)

import nv_utils as utils  # 导入工具模块


def nv_build(config: dict):
    """
    主构建函数
    """
    # ================= 1. 参数解析与默认值 =================
    project_dir = config.get("project_dir", "ne_vision")
    build_dir = config.get("build_dir", "build")
    install_dir = config.get("install_dir", "install")
    build_type = config.get("build_type", "Release")
    extra_cmake_args = config.get("extra_cmake_args", [])

    # GDExtension 相关配置
    build_gd = config.get("build_gd", False)
    ne_vision_gd_src = config.get("ne_vision_gd_dir")
    gd_extension_dst = config.get("gd_extension_dir")

    # 状态追踪
    status_cmake_build = False
    status_gd_deploy = "SKIPPED"  # 状态: SKIPPED, SUCCESS, FAILED

    # ================= 2. 输出配置日志 =================
    utils.log_process("Initializing Build Configuration...")
    utils.log_info("Project Dir", project_dir)
    utils.log_info("Build Dir", build_dir)
    utils.log_info("Install Dir", install_dir)
    utils.log_info("Build Type", build_type)
    utils.log_info("Extra Args", extra_cmake_args)
    utils.log_info("GDExtension", build_gd)
    if build_gd:
        utils.log_info(" -> Source", ne_vision_gd_src)
        utils.log_info(" -> Dest", gd_extension_dst)
    print("-" * 60)

    # ================= 3. 环境准备 =================
    if not os.path.exists(project_dir):
        utils.log_error(f"Project directory not found: {project_dir}")

    cmake_gen_args = []
    build_env = {}

    # 工具链检测
    if utils.check_tool("ninja"):
        utils.log_process("Ninja detected. Using Ninja generator.")
        cmake_gen_args.extend(["-G", "Ninja"])
    else:
        utils.log_process("Ninja not found. Using default generator.")

    if utils.check_tool("clang") and utils.check_tool("clang++"):
        utils.log_process("Clang/Clang++ detected. Setting as compiler.")
        build_env["CC"] = "clang"
        build_env["CXX"] = "clang++"
    else:
        utils.log_process("Clang not found. Using system default compiler.")

    if not os.path.exists(build_dir):
        os.makedirs(build_dir)

    # ================= 4. CMake 构建流程 =================
    try:
        # [Step A] Configure
        utils.log_process("Step 1/3: Configuring (CMake)...")
        config_cmd = [
            "cmake",
            "-S",
            project_dir,
            "-B",
            build_dir,
            f"-DCMAKE_INSTALL_PREFIX={install_dir}",
            f"-DCMAKE_BUILD_TYPE={build_type}",
        ]
        config_cmd.extend(cmake_gen_args)
        config_cmd.extend(extra_cmake_args)
        utils.run_command(config_cmd, env=build_env)

        # [Step B] Build
        utils.log_process("Step 2/3: Building (CMake)...")
        build_cmd = ["cmake", "--build", build_dir, "--parallel"]
        utils.run_command(build_cmd)

        # [Step C] Install
        utils.log_process("Step 3/3: Installing (CMake)...")
        install_cmd = ["cmake", "--install", build_dir]
        utils.run_command(install_cmd)

        status_cmake_build = True

    except Exception as e:
        # 注意：utils.run_command 默认会直接 exit(1)，
        # 如果这里能捕获到异常，说明是其他 python 逻辑错误
        utils.log_error(f"Build process crashed: {e}")

    # ================= 5. GDExtension 部署逻辑 =================
    if build_gd:
        is_src_valid = ne_vision_gd_src and os.path.exists(ne_vision_gd_src)
        is_dst_valid = gd_extension_dst is not None

        if is_src_valid and is_dst_valid:
            utils.log_process("Deploying GDExtension...")
            try:
                # 1. 计算包含文件夹名的完整目标路径
                # 例如: src="ne_vision/gd_ext", dst="addons/" -> target="addons/gd_ext"
                src_folder_name = os.path.basename(os.path.normpath(ne_vision_gd_src))
                final_target_path = os.path.join(gd_extension_dst, src_folder_name)

                # 2. 清理旧版本 (针对最终的子文件夹进行清理)
                if os.path.exists(final_target_path):
                    utils.log_process(f"Cleaning old extension: {final_target_path}")
                    shutil.rmtree(final_target_path)

                # 3. 确保父级目标目录存在 (例如 addons 文件夹)
                if not os.path.exists(gd_extension_dst):
                    os.makedirs(gd_extension_dst)

                # 4. 复制 (现在 copytree 会创建 final_target_path 这一级目录)
                utils.log_process(f"Copying: {ne_vision_gd_src} -> {final_target_path}")
                shutil.copytree(ne_vision_gd_src, final_target_path)

                status_gd_deploy = "SUCCESS"

            except Exception as e:
                utils.log_error(f"GDExtension deployment failed: {e}", exit_code=None)
                status_gd_deploy = "FAILED (Copy Error)"
        else:
            if not is_src_valid:
                utils.log_warning(f"GDExtension source invalid: {ne_vision_gd_src}")
            if not is_dst_valid:
                utils.log_warning("GDExtension destination not defined.")
            status_gd_deploy = "FAILED (Config Error)"

    # ================= 6. 最终结果汇总 =================
    print("\n" + "=" * 20 + " BUILD SUMMARY " + "=" * 20)

    # C++ Core Status
    core_msg = "SUCCESS" if status_cmake_build else "FAILED"
    core_color = "\033[1;32m" if status_cmake_build else "\033[1;31m"
    print(f"Core Project ({project_dir}): {core_color}{core_msg}\033[0m")

    # GDExtension Status
    if build_gd:
        gd_color = "\033[1;32m" if status_gd_deploy == "SUCCESS" else "\033[1;31m"
        print(f"GDExtension Deployment:      {gd_color}{status_gd_deploy}\033[0m")

        # 如果部署失败，额外提示
        if status_gd_deploy != "SUCCESS":
            print(
                f"\033[1;33m[Suggestion] Check paths or file permissions for GDExtension.\033[0m"
            )
    else:
        print(f"GDExtension Deployment:      SKIPPED")

    # ================= 7. Post-build instructions =================
    setup_script = os.path.join(install_dir, "setup.sh")
    if status_cmake_build and os.path.exists(setup_script):
        print("\n" + "=" * 18 + " POST-BUILD ACTION " + "=" * 18)
        print(f"\033[1;32m[ACTION]\033[0m To configure your environment, run:\n")
        print(f"    source {os.path.abspath(setup_script)}\n")

    print("=" * 55 + "\n")

    # 如果有任何严重的失败，以非0状态码退出
    if not status_cmake_build or (build_gd and status_gd_deploy != "SUCCESS"):
        sys.exit(1)
