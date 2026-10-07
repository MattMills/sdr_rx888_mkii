cmake_minimum_required(VERSION 3.13)
project(rx888_mkii_source CXX)

# Out-of-tree SDR++ source module for the RX888 mkII, plus a standalone
# command-line test tool (rx888_tool) that shares the device and DSP code.
#
#   SDRPP_SOURCE_DIR  path to the SDRPlusPlus source tree
#   SDRPP_BUILD_DIR   path to an existing SDR++ build of that tree
#                     (must contain the sdrpp_core import library)
#
# If SDRPP_SOURCE_DIR is not set only rx888_tool is built.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif ()

set(SDRPP_SOURCE_DIR "" CACHE PATH "SDRPlusPlus source tree")
set(SDRPP_BUILD_DIR "" CACHE PATH "SDRPlusPlus build tree containing sdrpp_core")
option(RX888_BUILD_TOOL "Build the rx888_tool command line test utility" ON)

# ---------------------------------------------------------------- dependencies
find_package(PkgConfig QUIET)
if (PKG_CONFIG_FOUND AND NOT WIN32)
    pkg_check_modules(PC_LIBUSB QUIET libusb-1.0)
    pkg_check_modules(PC_FFTW3F QUIET fftw3f)
    pkg_check_modules(PC_VOLK QUIET volk)
endif ()

find_path(LIBUSB_INCLUDE_DIR libusb.h PATH_SUFFIXES libusb-1.0 HINTS ${PC_LIBUSB_INCLUDE_DIRS})
find_library(LIBUSB_LIBRARY NAMES usb-1.0 libusb-1.0 HINTS ${PC_LIBUSB_LIBRARY_DIRS})
if (NOT LIBUSB_INCLUDE_DIR OR NOT LIBUSB_LIBRARY)
    message(FATAL_ERROR "libusb-1.0 not found (Windows: vcpkg libusb or PothosSDR; Linux: libusb-1.0-0-dev; macOS: brew install libusb)")
endif ()

find_package(FFTW3f CONFIG QUIET)
if (TARGET FFTW3::fftw3f)
    set(RX888_FFTW_TARGET FFTW3::fftw3f)
else ()
    find_path(FFTW3F_INCLUDE_DIR fftw3.h HINTS ${PC_FFTW3F_INCLUDE_DIRS})
    find_library(FFTW3F_LIBRARY NAMES fftw3f libfftw3f-3 fftw3f-3 HINTS ${PC_FFTW3F_LIBRARY_DIRS})
    if (NOT FFTW3F_INCLUDE_DIR OR NOT FFTW3F_LIBRARY)
        message(FATAL_ERROR "fftw3f not found")
    endif ()
    add_library(rx888_fftw3f INTERFACE)
    target_include_directories(rx888_fftw3f INTERFACE ${FFTW3F_INCLUDE_DIR})
    target_link_libraries(rx888_fftw3f INTERFACE ${FFTW3F_LIBRARY})
    set(RX888_FFTW_TARGET rx888_fftw3f)
endif ()

find_package(Threads REQUIRED)

# ------------------------------------------------------------ embedded firmware
set(FIRMWARE_IMG "${CMAKE_CURRENT_SOURCE_DIR}/firmware/SDDC_FX3.img")
set(FIRMWARE_CPP "${CMAKE_CURRENT_BINARY_DIR}/sddc_fx3_firmware.cpp")
add_custom_command(
    OUTPUT ${FIRMWARE_CPP}
    COMMAND ${CMAKE_COMMAND} -DINPUT=${FIRMWARE_IMG} -DOUTPUT=${FIRMWARE_CPP} -DSYMBOL=sddc_fx3_firmware
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_file.cmake
    DEPENDS ${FIRMWARE_IMG} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_file.cmake
    COMMENT "Embedding SDDC_FX3.img")

# --------------------------------------------- device + DSP library (no SDR++)
add_library(rx888_core STATIC
    src/device/fx3_device.cpp
    src/device/rx888_mk2.cpp
    src/dsp/kaiser.cpp
    src/dsp/r2iq.cpp
    ${FIRMWARE_CPP})
target_include_directories(rx888_core PUBLIC src ${LIBUSB_INCLUDE_DIR})
target_link_libraries(rx888_core PUBLIC ${LIBUSB_LIBRARY} ${RX888_FFTW_TARGET} Threads::Threads)
set_target_properties(rx888_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
if (MSVC)
    target_compile_options(rx888_core PRIVATE /O2 /W3 /EHsc)
    target_compile_definitions(rx888_core PUBLIC _CRT_SECURE_NO_WARNINGS NOMINMAX)
else ()
    target_compile_options(rx888_core PRIVATE -O3 -Wall)
endif ()

if (RX888_BUILD_TOOL)
    add_executable(rx888_tool tools/rx888_tool.cpp)
    target_link_libraries(rx888_tool PRIVATE rx888_core)
    install(TARGETS rx888_tool DESTINATION bin)
endif ()

if (UNIX AND NOT APPLE)
    install(FILES packaging/linux/99-rx888-mkii.rules DESTINATION lib/udev/rules.d)
endif ()

# ------------------------------------------------------------ SDR++ module
if (SDRPP_SOURCE_DIR)
    if (NOT SDRPP_BUILD_DIR)
        message(FATAL_ERROR "SDRPP_BUILD_DIR must point at an SDR++ build of ${SDRPP_SOURCE_DIR}")
    endif ()

    # Import sdrpp_core from the existing SDR++ build.
    add_library(sdrpp_core SHARED IMPORTED)
    if (WIN32)
        find_file(SDRPP_CORE_IMPLIB sdrpp_core.lib
            PATHS "${SDRPP_BUILD_DIR}/core/Release" "${SDRPP_BUILD_DIR}/core/RelWithDebInfo" "${SDRPP_BUILD_DIR}/core"
            NO_DEFAULT_PATH)
        find_file(SDRPP_CORE_DLL sdrpp_core.dll
            PATHS "${SDRPP_BUILD_DIR}/core/Release" "${SDRPP_BUILD_DIR}/core/RelWithDebInfo" "${SDRPP_BUILD_DIR}/core"
            NO_DEFAULT_PATH)
        if (NOT SDRPP_CORE_IMPLIB)
            message(FATAL_ERROR "sdrpp_core.lib not found under ${SDRPP_BUILD_DIR}/core")
        endif ()
        set_target_properties(sdrpp_core PROPERTIES IMPORTED_IMPLIB "${SDRPP_CORE_IMPLIB}" IMPORTED_LOCATION "${SDRPP_CORE_DLL}")
    else ()
        find_library(SDRPP_CORE_LIB NAMES sdrpp_core PATHS "${SDRPP_BUILD_DIR}/core" "${SDRPP_BUILD_DIR}" NO_DEFAULT_PATH)
        if (NOT SDRPP_CORE_LIB)
            message(FATAL_ERROR "libsdrpp_core not found under ${SDRPP_BUILD_DIR}/core")
        endif ()
        set_target_properties(sdrpp_core PROPERTIES IMPORTED_LOCATION "${SDRPP_CORE_LIB}")
    endif ()
    set_target_properties(sdrpp_core PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
        "${SDRPP_SOURCE_DIR}/core/src;${SDRPP_SOURCE_DIR}/core/src/imgui;${SDRPP_SOURCE_DIR}/core/backends/glfw;${SDRPP_SOURCE_DIR}/core/backends/glfw/imgui")

    # SDR++ core headers include <volk/volk.h> and call into volk inline
    # (in-tree modules get this through sdrpp_core's link interface).
    find_package(Volk CONFIG QUIET)
    find_path(VOLK_INCLUDE_DIR volk/volk.h HINTS ${VOLK_INCLUDE_DIRS} ${PC_VOLK_INCLUDE_DIRS})
    find_library(VOLK_LIBRARY NAMES volk libvolk HINTS ${PC_VOLK_LIBRARY_DIRS})
    if (NOT VOLK_INCLUDE_DIR OR NOT VOLK_LIBRARY)
        message(FATAL_ERROR "volk not found (set VOLK_INCLUDE_DIR/VOLK_LIBRARY or CMAKE_PREFIX_PATH)")
    endif ()

    add_library(rx888_mkii_source SHARED src/main.cpp)
    target_link_libraries(rx888_mkii_source PRIVATE sdrpp_core rx888_core ${VOLK_LIBRARY})
    target_include_directories(rx888_mkii_source PRIVATE ${VOLK_INCLUDE_DIR})
    set_target_properties(rx888_mkii_source PROPERTIES PREFIX "")
    if (MSVC)
        # Same flags SDR++ uses for in-tree modules (SDRPP_MODULE_COMPILER_FLAGS).
        target_compile_options(rx888_mkii_source PRIVATE /O2 /Ob2 /std:c++17 /EHsc)
    else ()
        target_compile_options(rx888_mkii_source PRIVATE -O3 -std=c++17)
    endif ()
    install(TARGETS rx888_mkii_source DESTINATION lib/sdrpp/plugins)
endif ()
