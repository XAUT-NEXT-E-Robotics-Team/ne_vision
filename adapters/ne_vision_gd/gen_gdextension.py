import os
import platform
import sys
from pathlib import PurePath

# ANSI color codes for terminal output
RED = "\033[91m"
GREEN = "\033[92m"
RESET = "\033[0m"
PREFIX = "[ne_vision][GDExtension] "


def is_path_syntax(val):
    try:
        path = PurePath(val)
        return len(path.parts) > 1 or path.suffix != "" or path.is_absolute()
    except Exception:
        return False


def log_error(message):
    """Prints an error message in red with the required prefix."""
    print(f"{PREFIX}{RED}Error: {message}{RESET}")


def log_info(message):
    """Prints an informational message in green with the required prefix."""
    print(f"{PREFIX}{GREEN}{message}{RESET}")


def generate_gdextension(output_file_path):
    # Detect system details
    current_os = platform.system().lower()
    current_machine = platform.machine().lower()

    # --- Step 1: Platform Validation & Extension Mapping ---
    # Godot uses 'macos' and 'linux' as keys in .gdextension files
    if current_os == "linux":
        gd_os_key = "linux"
        lib_ext = "so"
    elif current_os == "darwin":
        gd_os_key = "macos"
        lib_ext = "dylib"
    else:
        log_error(
            f"Unsupported Operating System: {platform.system()}. This extension only supports Linux and macOS."
        )
        sys.exit(1)

    # --- Step 2: Architecture Validation ---
    arch_key = ""
    if current_machine in ["x86_64", "amd64"]:
        arch_key = "x86_64"
    elif current_machine in ["aarch64", "arm64"]:
        arch_key = "arm64"
    else:
        log_error(
            f"Unsupported Architecture: {current_machine}. Only x86_64 and ARM64 are supported."
        )
        sys.exit(1)

    # --- Step 3: Configuration Data ---
    entry_symbol = "ne_vision_gd_library_init"
    # 注意：4.5 为假设的未来版本或特定需求，通常生产环境用 4.1 或 4.2
    min_compatibility = "4.5"

    # 构造库路径逻辑
    # 路径格式: lib/libne_vision_gd.[arch].[ext]
    lib_relative_path = f"lib/libne_vision_gd.{arch_key}.{lib_ext}"

    lib_lines = [
        f'{gd_os_key}.debug.{arch_key} = "{lib_relative_path}"',
        f'{gd_os_key}.release.{arch_key} = "{lib_relative_path}"',
    ]

    # --- Step 4: Content Assembly ---
    content = [
        "[configuration]",
        f'entry_symbol = "{entry_symbol}"',
        f'compatibility_minimum = "{min_compatibility}"',
        "reloadable = true",
        "",
        "[libraries]",
        *lib_lines,
    ]

    # --- Step 5: File Writing ---
    try:
        # 使用传入的参数作为写入路径
        with open(output_file_path, "w", encoding="utf-8") as f:
            f.write("\n".join(content))

        log_info("Configuration generated successfully!")
        log_info(f"Target OS: {gd_os_key.capitalize()}")
        log_info(f"Architecture: {arch_key}")
        log_info(f"Output: {os.path.abspath(output_file_path)}")
    except Exception as e:
        log_error(f"Failed to write file: {e}")
        sys.exit(1)


if __name__ == "__main__":
    # 优先使用命令行参数指定的路径
    if len(sys.argv) > 1 and is_path_syntax(str(sys.argv[1])):
        generate_gdextension(str(sys.argv[1]))
    else:
        # 回退到默认文件名
        log_info("No path provided, using default filename: ne_vision_gd.gdextension")
        generate_gdextension("ne_vision_gd.gdextension")
