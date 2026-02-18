# 兼容 bash 和 zsh 的写法
if [ -n "$ZSH_VERSION" ]; then
    # zsh 环境
    SCRIPT_DIR=$(cd "$(dirname "${(%):-%N}")" && pwd)
elif [ -n "$BASH_VERSION" ]; then
    # bash 环境
    SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
else
    # 其他 sh 尝试
    SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
fi

export NE_VISION_INSTALL_PATH="$SCRIPT_DIR"
echo "NE_VISION_INSTALL_PATH: $NE_VISION_INSTALL_PATH"
