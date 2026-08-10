# 3.15+ required to enable MSVC_RUNTIME_LIBRARY (CMP0091);
# CMake 4.x refuses to configure projects declaring a minimum below 3.5
cmake_minimum_required(VERSION 3.15)
cmake_policy(SET CMP0091 NEW)
# set(CMAKE_POLICY_DEFAULT_CMP0091 NEW)

project(ExtIO_RTL)

### build options
# default build type: Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE)
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE} " - Version: " ${VERSION} " / " ${LIBVER})

if (NOT MSVC)
    message(ERROR "CMakeLists.txt does only support the MSVC compiler")
endif()


# allow overriding cmake options with standard variables - from a project including this one
cmake_policy(SET CMP0077 NEW)  # set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

set(RTL_STATIC_BUILD TRUE)
set(LINK_RTLTOOLS_AGAINST_STATIC_LIB TRUE)

if (WIN32)
    set( EXTIO_THREAD_LIB libpthreadVC3 )
    set(PTHREADS4W_VERSION 3)
    add_subdirectory(pthread-win32)
    if (TARGET libpthreadVC3)
        set( THREADS_FOUND  TRUE)
        set( Threads_FOUND  TRUE)
        set( THREADS_PTHREADS_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/pthread-win32" )
        set( THREADS_PTHREADS_WIN32_LIBRARY libpthreadVC3)
        set( CMAKE_THREAD_LIBS_INIT  libpthreadVC3)
    else()
        message(FATAL_ERROR "ExtIO_RTL: target libpthreadVC3 not defined!")
    endif()

    set( EXTIO_USB_LIB libusb_static )
    add_subdirectory(libusb)
    if (TARGET libusb_static)
        set( LIBUSB_FOUND TRUE )
        set( LIBUSB_LIBRARIES  libusb_static )
        if (MSVC)
            # libusb enables /WX; newer MSVC versions add warnings (e.g. C5286)
            # that break its build - the last /WX- on the command line wins
            target_compile_options(libusb_static PRIVATE /WX-)
        endif()
    endif()
else()
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    set( EXTIO_THREAD_LIB Threads::Threads )

    set( EXTIO_USB_LIB libusb )
endif()

add_subdirectory(librtlsdr)


add_library(ExtIO_RTL SHARED EXCLUDE_FROM_ALL
    src/control_tcp.cpp
    src/control.h
    src/ExtIO_RTL.cpp
    src/ExtIO_RTL.h
    src/LC_ExtIO_Types.h
    src/config_file.cpp
    src/config_file.h
    src/dllmain.cpp
    src/ExtIO_RTL.def
    src/resource.h
    src/targetver.h
    src/tuners.h
    src/tuners.cpp
    src/rates.h
    src/rates.cpp
    src/gui_dlg.cpp
    src/gui_dlg.h
)

set_target_properties(ExtIO_RTL PROPERTIES PREFIX "")
set_property(TARGET ExtIO_RTL PROPERTY CXX_STANDARD 17)
set_property(TARGET ExtIO_RTL PROPERTY CXX_STANDARD_REQUIRED ON)
target_compile_definitions(ExtIO_RTL PUBLIC _CRT_SECURE_NO_WARNINGS)
target_compile_definitions(ExtIO_RTL PRIVATE rtlsdr_STATIC)

if (MSVC)
    target_sources(ExtIO_RTL PRIVATE
        src/ExtIO_RTL.rc
    )
    target_compile_definitions(ExtIO_RTL PRIVATE HAS_WIN_GUI_DLG)

    # set our and override dependent libraries' MSVC_RUNTIME_LIBRARY
    set_property(TARGET ExtIO_RTL PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    set_property(TARGET rtlsdr_static PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    set_property(TARGET ${EXTIO_THREAD_LIB} PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    set_property(TARGET ${EXTIO_USB_LIB} PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()


target_include_directories(ExtIO_RTL PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    tomlplusplus/include
    librtlsdr/include
)

target_compile_definitions(rtlsdr_static PUBLIC _CRT_SECURE_NO_WARNINGS)

target_link_libraries(ExtIO_RTL PRIVATE
    rtlsdr_static
    ${EXTIO_THREAD_LIB}
    ${EXTIO_USB_LIB}
)


set(RTLTOOLS "rtl_sdr;rtl_tcp;rtl_udp;rtl_test;rtl_eeprom;rtl_biast")
set(RTLTOOLS "${RTLTOOLS};rtl_fm")
# set(RTLTOOLS "${RTLTOOLS};rtl_multichannel")
set(RTLTOOLS "${RTLTOOLS};rtl_ir;rtl_adsb;rtl_power")
set(RTLTOOLS "${RTLTOOLS};rtl_raw2wav;rtl_wavestat;rtl_wavestream")

if (MSVC)
    # override tools' MSVC_RUNTIME_LIBRARY and define _CRT_SECURE_NO_WARNINGS
    set_property(TARGET convenience_static PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    target_compile_definitions(convenience_static PUBLIC _CRT_SECURE_NO_WARNINGS)
    set_property(TARGET libgetopt_static PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    target_compile_definitions(libgetopt_static PUBLIC _CRT_SECURE_NO_WARNINGS)
    foreach (tool ${RTLTOOLS})
        set_property(TARGET ${tool} PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
        target_compile_definitions(${tool} PUBLIC _CRT_SECURE_NO_WARNINGS)
    endforeach()
endif()
foreach (tool ${RTLTOOLS})
    set_target_properties(${tool} PROPERTIES EXCLUDE_FROM_ALL TRUE)
endforeach()

add_custom_target(rtl_tools
  DEPENDS ${RTLTOOLS}
)
