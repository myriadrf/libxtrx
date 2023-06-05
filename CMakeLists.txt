project(libxtrx C)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

cmake_minimum_required(VERSION 2.8)


# Set the version information here
set(MAJOR_VERSION 0)
set(API_COMPAT    0)
set(MINOR_VERSION 1)
set(MAINT_VERSION 2)

set(LIBVER  "${MAJOR_VERSION}.${API_COMPAT}.${MINOR_VERSION}")

# determine target architecture
execute_process(COMMAND ${CMAKE_C_COMPILER} -print-multiarch OUTPUT_VARIABLE CC_ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)
if(CC_ARCH MATCHES "i386")
	message(STATUS "Building for i386")
	set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "i386")
	set(ARCH "x86")
elseif(CC_ARCH MATCHES "x86_64")
	message(STATUS "Building for x86_64")
	set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
	set(ARCH "x86")
elseif(CC_ARCH MATCHES "aarch64")
	message(STATUS "Building for aarch64")
	set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
	set(ARCH "arm")
elseif(CC_ARCH MATCHES "arm.*eabihf")
	message(STATUS "Building for armhf")
	set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
	set(ARCH "arm")
else()
	message(WARNING "Unknown target architecture ${CC_ARCH}, forcing to FORCE_ARCH=${FORCE_ARCH} variable if it was provided")
	set(ARCH ${FORCE_ARCH})
endif()

set(CROSS_COMPILE_INCLUDE_PATH "/usr/${CC_ARCH}/include")
set(CROSS_COMPILE_LIB_PATH     "/usr/${CC_ARCH}/lib")


if(CMAKE_SYSTEM_NAME MATCHES "Linux")
	set(SYSTEM_LIBS rt dl pthread)
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
	set(SYSTEM_LIBS "")
	if(CMAKE_COMPILER_IS_GNUCC)
		set(CMAKE_SHARED_LINKER_FLAGS " -Wl,--kill-at -static-libgcc ")
	endif(CMAKE_COMPILER_IS_GNUCC)
elseif(CMAKE_SYSTEM_NAME MATCHES "Darwin")
	set(SYSTEM_LIBS rt dl pthread)
else()
	message(WARNING "Unknown system type is ${CMAKE_SYSTEM_NAME}")
endif()

add_definitions(-Wall -g)

set(XTRX_LIBRARY_DIR      lib${LIB_SUFFIX})
set(XTRX_INCLUDE_DIR      include)
set(XTRX_UTILS_DIR        bin)

CONFIGURE_FILE(
	${CMAKE_CURRENT_SOURCE_DIR}/libxtrx.pc.in
	${CMAKE_CURRENT_BINARY_DIR}/libxtrx.pc
	@ONLY)

INSTALL(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/libxtrx.pc
	DESTINATION ${XTRX_LIBRARY_DIR}/pkgconfig)

# Report all undfined symbols in shared libraries
if(CMAKE_COMPILER_IS_GNUCC)
	add_definitions("-Wl,-z,defs")
endif(CMAKE_COMPILER_IS_GNUCC)

find_package(libxtrxdsp REQUIRED)
find_package(libxtrxll REQUIRED)
find_package(liblms7compact REQUIRED)

include_directories(${LIBXTRXDSP_INCLUDE_DIRS})
include_directories(${LIBXTRXLL_INCLUDE_DIRS})
include_directories(${LIBLMS7COMPACT_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(xtrx SHARED
	xtrx.c
	xtrx_debug.c
	xtrx_fe.c
	xtrx_fe_nlms7.c
	xtrx_fe_octorx0.c
	xtrx_fe_octocal0.c
	octo/adf4355.c
	)
target_link_libraries(xtrx ${LIBXTRXLL_LIBRARIES} ${LIBXTRXDSP_LIBRARIES} ${LIBLMS7COMPACT_LIBRARIES} m)

set_target_properties(xtrx PROPERTIES VERSION ${LIBVER} SOVERSION ${MAJOR_VERSION})

add_executable(test_xtrx test_xtrx.c)
target_link_libraries(test_xtrx xtrx ${SYSTEM_LIBS})

########################################################################
# XTRX QT examples
########################################################################
find_package(Qt5Widgets)
find_package(Qt5PrintSupport)
find_package(QCustomPlot)
if(Qt5Widgets_FOUND AND Qt5PrintSupport_FOUND AND QCustomPlot_FOUND)
    set(LIBXTRX_FOUND TRUE "libxtrx found")
    set(LIBXTRX_LIBRARIES xtrx)
    set(LIBXTRX_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
    add_subdirectory(examples/xtrx_fft)
endif()

########################################################################
# install libraries
########################################################################
install(TARGETS xtrx DESTINATION ${XTRX_LIBRARY_DIR})
install(TARGETS test_xtrx DESTINATION ${XTRX_UTILS_DIR})

########################################################################
# install headers
########################################################################
install(FILES
	xtrx_api.h
	DESTINATION ${XTRX_INCLUDE_DIR})

option(ENABLE_SOAPY "Enable SOAPY plugin for XTRX" ON)
if(ENABLE_SOAPY)
	find_package(SoapySDR)
	if(SoapySDR_FOUND)
		add_subdirectory(soapy)
	else()
		message(WARNING "SoapySDR hasn't been found -- ignoring SOAPY plugin")
	endif()
endif(ENABLE_SOAPY)

########################################################################
# package generator
########################################################################
if(NOT CPACK_GENERATOR)
	set(CPACK_GENERATOR DEB)
endif()
set(CPACK_PACKAGE_NAME                "libxtrx")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "XTRX API library")
set(CPACK_PACKAGE_VENDOR              "Fairwaves, Inc.")
set(CPACK_PACKAGE_CONTACT             "http://fairwaves.co/wp/contact-us/")
set(CPACK_PACKAGE_VERSION             ${LIBVER}-${MAINT_VERSION})
set(CPACK_DEBIAN_PACKAGE_DEPENDS      "libc6 (>= 2.17), libxtrxll (>= 0.0.1), libxtrxdsp (>= 0.0.1), liblms7002m (>= 0.0.1)")
if(ENABLE_SOAPY)
#	TODO: need a dependency for correct soapy version
#	set(CPACK_DEBIAN_PACKAGE_DEPENDS      "${CPACK_DEBIAN_PACKAGE_DEPENDS}, soapysdr (>= 0.5.2)")
endif(ENABLE_SOAPY)


set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_SET_DESTDIR               "")
set(CPACK_PACKAGING_INSTALL_PREFIX  "/usr")
set(CPACK_PACKAGE_CONTACT           "Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>")

include(CPack) #include last
