# 选项：即使在 SDK 可用时也强制使用 stub（用于测试编译）
option(DAHENG_TEST_NO_STUB "Test building dahengcamera without stub (requires Daheng SDK)" OFF)

# Daheng GxIAPI 目前只提供 Linux x86_64 库；其他平台回退到 stub
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(DAHENG_SDK_LIB_DIR "${DAHENG_SDK_ROOT}/lib")
    if (EXISTS "${DAHENG_SDK_LIB_DIR}/libgxiapi.so")
        set(DAHENG_SDK_FOUND TRUE)
    else ()
        set(DAHENG_SDK_FOUND FALSE)
    endif ()
else ()
    set(DAHENG_SDK_FOUND FALSE)
endif ()

# 决定是否使用 SDK
if (DAHENG_SDK_FOUND OR DAHENG_TEST_NO_STUB)
    message(STATUS "Building dahengcamera with Daheng SDK for ${CMAKE_SYSTEM_PROCESSOR} (lib: ${DAHENG_SDK_LIB_DIR})")
    set(DAHENG_USE_SDK TRUE)
else ()
    if (DAHENG_FORCE_STUB)
        message(STATUS "Building dahengcamera with stub (forced by DAHENG_FORCE_STUB)")
    elseif (NOT DAHENG_SDK_FOUND)
        message(STATUS "Building dahengcamera with stub (Daheng SDK not available for ${CMAKE_SYSTEM_PROCESSOR} or lib directory not found)")
    endif ()
    set(DAHENG_USE_SDK FALSE)
endif ()

if (DAHENG_USE_SDK)
    crate(
        NAME dahengcamera
        TYPE SHARED
        SOURCES
            src/daheng_camera.cpp
        DEPS
            magic_enum
            talos_math
            fmt::fmt
            talos_log
            opencv_core
            opencv_imgproc
    )

    target_include_directories(dahengcamera SYSTEM PUBLIC ${DAHENG_SDK_ROOT}/include)

    target_link_directories(dahengcamera PRIVATE ${DAHENG_SDK_LIB_DIR})

    target_link_libraries(dahengcamera PRIVATE
        gxiapi
    )

    # Set rpath to find the SDK libraries at runtime
    set_target_properties(dahengcamera PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${DAHENG_SDK_LIB_DIR}"
        BUILD_WITH_INSTALL_RPATH TRUE
    )

    # Suppress warnings from external SDK headers
    crate_silence_warnings(dahengcamera)
else ()
    crate(
        NAME dahengcamera
        TYPE SHARED
        SOURCES
            src/daheng_camera_stub.cpp
        DEPS
            magic_enum
            talos_math
            talos_log
            fmt::fmt
            opencv_core
            opencv_imgproc
    )
endif ()
