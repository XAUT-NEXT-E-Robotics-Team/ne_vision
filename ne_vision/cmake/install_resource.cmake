cmake_minimum_required(VERSION 3.16)

# ==============================================================================
# 函数: install_resource
# 描述: 根据全局开关 USE_SYMLINK_INSTALL 自动选择 直接复制 或 创建软链接
# 参数:
#   SOURCE      : 资源源目录 (例如 ${CMAKE_SOURCE_DIR}/assets)
#   DESTINATION : 安装目标路径 (相对于 CMAKE_INSTALL_PREFIX，例如 share/myapp)
# ==============================================================================
function(install_resource)
    # 1. 解析参数
    set(options)
    set(oneValueArgs SOURCE DESTINATION)
    set(multiValueArgs)
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT ARG_SOURCE OR NOT ARG_DESTINATION)
        message(FATAL_ERROR "Usage: install_smart_resource(SOURCE <dir> DESTINATION <dir>)")
    endif()

    # 2. 获取源目录的绝对路径和目录名
    get_filename_component(ABS_SRC "${ARG_SOURCE}" ABSOLUTE)
    get_filename_component(DIR_NAME "${ABS_SRC}" NAME) # 获取最后一级目录名，如 "assets"

    # 3. 核心逻辑分支
    if(USE_SYMLINK_INSTALL)
        # === 模式 A: 软链接 (Symlink) ===
        # 注意：这里我们通过字符串拼接将变量“烧录”进 install(CODE) 中
        # 因为 install(CODE) 在安装时运行，那时访问不到当前的局部变量

        install(CODE "
            # [安装时逻辑]
            set(SRC_PATH \"${ABS_SRC}\")
            # 目标全路径 = 安装前缀 + 目标相对路径 + 目录名
            set(DEST_PARENT \"\${CMAKE_INSTALL_PREFIX}/${ARG_DESTINATION}\")
            set(DEST_PATH   \"\${DEST_PARENT}/${DIR_NAME}\")

            message(STATUS \"[Symlink] \${SRC_PATH} -> \${DEST_PATH}\")

            # 1. 创建父目录
            file(MAKE_DIRECTORY \"\${DEST_PARENT}\")

            # 2. 如果目标存在（可能是旧的实体文件夹），先递归删除，否则无法创建链接
            if(EXISTS \"\${DEST_PATH}\")
                # 检查是否已经是软链接，如果是且指向正确则跳过（可选优化，这里为了稳健每次都重做）
                file(REMOVE_RECURSE \"\${DEST_PATH}\")
            endif()

            # 3. 创建链接 (CMake 3.14+)
            file(CREATE_LINK \"\${SRC_PATH}\" \"\${DEST_PATH}\" SYMBOLIC)
        ")
    else()
        # === 模式 B: 直接复制 (Copy) ===
        message(STATUS "[Configure] Plan to COPY resources: ${DIR_NAME}")

        install(DIRECTORY "${ARG_SOURCE}"
            DESTINATION "${ARG_DESTINATION}"
            USE_SOURCE_PERMISSIONS
        )
    endif()
endfunction()
