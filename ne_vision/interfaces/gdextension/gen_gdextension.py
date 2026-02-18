import os
import platform
import sys
from pathlib import PurePath

# ANSI color codes for terminal output
RED = "\033[91m"
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
    """Prints an informational message with the required prefix."""
    print(f"{PREFIX}{message}")


def generate_gdextension(file_path):
    # Detect system details
    current_os = platform.system().lower()
    current_machine = platform.machine().lower()

    # --- Step 1: Platform Validation (Linux Only) ---
    if current_os != "linux":
        log_error(
            f"Unsupported Operating System: {platform.system()}. This extension only supports Linux."
        )
        sys.exit(1)

    # --- Step 2: Architecture Validation (x86_64 or ARM) ---
    # Standardizing architecture strings for Godot compatibility
    arch_key = ""
    if current_machine in ["x86_64", "amd64"]:
        arch_key = "x86_64"
    elif current_machine in ["aarch64", "arm64"]:
        arch_key = "arm64"
    elif "arm" in current_machine:
        arch_key = "arm32"
    else:
        log_error(
            f"Unsupported Linux Architecture: {current_machine}. Only x86_64 and ARM are supported."
        )
        sys.exit(1)

    # --- Step 3: Configuration Data ---
    file_name = "ne_vision_gd.gdextension"
    entry_symbol = "ne_vision_gd_library_init"
    min_compatibility = "4.5"

    # Define paths based on the detected architecture
    # These paths follow the convention: res://bin/linux/libne_vision.[debug/release].[arch].so
    lib_lines = [
        f'linux.debug.{arch_key} = "lib/libne_vision_gd.{arch_key}.so"',
        f'linux.release.{arch_key} = "lib/libne_vision_gd.{arch_key}.so"',
    ]

    # --- Step 4: Content Assembly ---
    content = [
        "[configuration]",
        f'entry_symbol = "{entry_symbol}"',
        f'compatibility_minimum = "{min_compatibility}"',
        "reloadable = true",
        "[libraries]",
        *lib_lines,
    ]

    # --- Step 5: File Writing ---
    try:
        with open(file_name, "w", encoding="utf-8") as f:
            f.write("\n".join(content))

        log_info("Configuration generated successfully!")
        log_info(f"Target: Linux {arch_key}")
        log_info(f"Path: {os.path.abspath(file_name)}")
    except Exception as e:
        log_error(f"Failed to write file: {e}")
        sys.exit(1)


if __name__ == "__main__":
    if len(sys.argv) > 1 and is_path_syntax(str(sys.argv[1])):
        generate_gdextension(str(sys.argv[1]))
    else:
        log_error(
            "File path argument is missing or invalid. Please provide a valid file path."
        )
