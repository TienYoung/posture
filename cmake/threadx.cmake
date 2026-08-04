set(THREADX_ROOT "${CMAKE_CURRENT_LIST_DIR}/../ThirdParty/threadx")
set(THREADX_APP_DIR "${CMAKE_CURRENT_LIST_DIR}/../AZURE_RTOS/App")

file(GLOB THREADX_COMMON_SOURCES CONFIGURE_DEPENDS
    "${THREADX_ROOT}/common/src/*.c"
)

file(GLOB THREADX_PORT_SOURCES CONFIGURE_DEPENDS
    "${THREADX_ROOT}/ports/cortex_m4/gnu/src/*.S"
)

add_library(threadx_kernel OBJECT
    ${THREADX_COMMON_SOURCES}
    ${THREADX_PORT_SOURCES}
)

target_include_directories(threadx_kernel PUBLIC
    "${THREADX_ROOT}/common/inc"
    "${THREADX_ROOT}/ports/cortex_m4/gnu/inc"
    "${THREADX_APP_DIR}"
    "${CMAKE_CURRENT_LIST_DIR}/../Core/Inc"
)

target_compile_definitions(threadx_kernel PUBLIC
    TX_INCLUDE_USER_DEFINE_FILE
)

target_link_libraries(threadx_kernel PUBLIC
    stm32cubemx
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    "${THREADX_APP_DIR}/app_azure_rtos.c"
    "${CMAKE_CURRENT_LIST_DIR}/../Core/Src/app_threadx.c"
    "${CMAKE_CURRENT_LIST_DIR}/../Core/Src/tx_initialize_low_level.s"
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    threadx_kernel
)
