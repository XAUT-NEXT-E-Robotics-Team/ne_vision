cmake_minimum_required(VERSION 3.16)

# ==============================================================================
# 函数: install_resource
# 描述: 根据全局开关 USE_SYMLINK_INSTALL 自动选择 直接复制 或 创建软链接
# 参数:
#   SOURCE      : 资源源目录 (例如 ${CMAKE_SOURCE_DIR}/assets)
#   DESTINATION : 安装目标路径 (相对于 BASE_DIR，例如 share/myapp)
#   BASE_DIR    : (可选) 安装基目录，默认使用 ${CMAKE_INSTALL_PREFIX}
# ==============================================================================
function(install_resource)
    # 1. 解析参数
    set(options)
    set(oneValueArgs SOURCE DESTINATION BASE_DIR)
    set(multiValueArgs)
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT ARG_SOURCE OR NOT ARG_DESTINATION)
        message(FATAL_ERROR "Usage: install_resource(SOURCE <path> DESTINATION <dir> [BASE_DIR <dir>])")
    endif()

    if(ARG_BASE_DIR)
        set(INSTALL_BASE "${ARG_BASE_DIR}")
    else()
        set(INSTALL_BASE "${CMAKE_INSTALL_PREFIX}")
    endif()

    # 2. 获取源路径的绝对路径和名称
    get_filename_component(ABS_SRC "${ARG_SOURCE}" ABSOLUTE)
    get_filename_component(OBJ_NAME "${ABS_SRC}" NAME)

    # 3. 检查源类型 (目录 or 文件)
    if(IS_DIRECTORY "${ABS_SRC}")
        set(IS_DIR TRUE)
    elseif(EXISTS "${ABS_SRC}")
        set(IS_DIR FALSE)
    else()
        # 如果文件/目录在配置阶段不存在，但也可能是生成阶段产生的文件
        # 我们可以根据是否带有扩展名或者以 / 结尾等启发式方法判断
        # 这里为了稳妥，如果既不是现有目录也不是存在的文件，并且没有后缀名，当作目录处理，否则当作文件处理。
        if(OBJ_NAME MATCHES "\\.[^.]+$")
            set(IS_DIR FALSE)
        else()
            set(IS_DIR TRUE)
        endif()
    endif()

    message(STATUS "[Configure] install_resource -> IS_DIR: ${IS_DIR} (${ABS_SRC})")

    # 4. 核心逻辑分支
    if(USE_SYMLINK_INSTALL)
        # === 模式 A: 软链接 (Symlink) ===
    if(IS_DIR)
        install(CODE "
            set(SRC_PATH \"${ABS_SRC}\")
            set(DEST_PARENT \"${INSTALL_BASE}/${ARG_DESTINATION}\")
            set(DEST_PATH   \"\${DEST_PARENT}/${OBJ_NAME}\")

            if(NOT EXISTS \"\${SRC_PATH}\")
                message(FATAL_ERROR \"Source directory does not exist, cannot create symlink: \${SRC_PATH}\")
            endif()

            message(STATUS \"[Symlink] \${SRC_PATH} -> \${DEST_PATH}\")

            # 1. 创建父目录
            file(MAKE_DIRECTORY \"\${DEST_PARENT}\")

            # 2. 存在则清理旧目标
            if(EXISTS \"\${DEST_PATH}\" OR IS_SYMLINK \"\${DEST_PATH}\")
                file(REMOVE_RECURSE \"\${DEST_PATH}\")
            endif()

            # 3. 创建符号链接
            file(CREATE_LINK \"\${SRC_PATH}\" \"\${DEST_PATH}\" SYMBOLIC)
        ")
    else()
        install(CODE "
            set(SRC_PATH \"${ABS_SRC}\")
            set(DEST_PARENT \"${INSTALL_BASE}/${ARG_DESTINATION}\")
            set(DEST_PATH   \"${INSTALL_BASE}/${ARG_DESTINATION}/${OBJ_NAME}\")

            if(NOT EXISTS \"\${SRC_PATH}\")
                message(FATAL_ERROR \"Source file does not exist, cannot create symlink: \${SRC_PATH}\")
            endif()

            message(STATUS \"[Symlink] \${SRC_PATH} -> \${DEST_PATH}\")

            # 1. 创建父目录
            file(MAKE_DIRECTORY \"\${DEST_PARENT}\")

            # 2. 存在则清理旧目标
            if(EXISTS \"\${DEST_PATH}\" OR IS_SYMLINK \"\${DEST_PATH}\")
                file(REMOVE \"\${DEST_PATH}\")
            endif()

            # 3. 创建符号链接
            file(CREATE_LINK \"\${SRC_PATH}\" \"\${DEST_PATH}\" SYMBOLIC)
        ")
    endif()
    else()
        # === 模式 B: 直接复制 (Copy) ===
        message(STATUS "[Configure] Plan to COPY resource: ${OBJ_NAME}")

        if(IS_DIR)
            install(CODE "
                set(SRC_PATH \"${ABS_SRC}\")
                set(DEST_PATH \"${INSTALL_BASE}/${ARG_DESTINATION}\")
                message(STATUS \"[Copy] \${SRC_PATH} -> \${DEST_PATH}\")
                file(COPY \"\${SRC_PATH}\" DESTINATION \"\${DEST_PATH}\" USE_SOURCE_PERMISSIONS)
            ")
        else()
            install(CODE "
                set(SRC_PATH \"${ABS_SRC}\")
                set(DEST_DIR \"${INSTALL_BASE}/${ARG_DESTINATION}\")
                message(STATUS \"[Copy] \${SRC_PATH} -> \${DEST_DIR}\")
                file(MAKE_DIRECTORY \"\${DEST_DIR}\")
                file(COPY \"\${SRC_PATH}\" DESTINATION \"\${DEST_DIR}\")
            ")
        endif()
    endif()
endfunction()
