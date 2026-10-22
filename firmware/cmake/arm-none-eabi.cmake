set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TOOLCHAIN_BIN
    "E:/DevTools/arm-gnu-toolchain-15.3.rel1-mingw-w64-i686-arm-none-eabi/bin"
)

set(CMAKE_C_COMPILER
    "${TOOLCHAIN_BIN}/arm-none-eabi-gcc.exe"
)

set(CMAKE_CXX_COMPILER
    "${TOOLCHAIN_BIN}/arm-none-eabi-g++.exe"
)

set(CMAKE_ASM_COMPILER
    "${TOOLCHAIN_BIN}/arm-none-eabi-gcc.exe"
)

set(CMAKE_AR
    "${TOOLCHAIN_BIN}/arm-none-eabi-ar.exe"
)

set(CMAKE_RANLIB
    "${TOOLCHAIN_BIN}/arm-none-eabi-ranlib.exe"
)

set(CMAKE_OBJCOPY
    "${TOOLCHAIN_BIN}/arm-none-eabi-objcopy.exe"
    CACHE FILEPATH ""
)

set(CMAKE_SIZE
    "${TOOLCHAIN_BIN}/arm-none-eabi-size.exe"
    CACHE FILEPATH ""
)

set(CMAKE_GDB
    "${TOOLCHAIN_BIN}/arm-none-eabi-gdb.exe"
    CACHE FILEPATH ""
)