cmake_minimum_required(VERSION 3.16)

project (modem VERSION 1.1.1)

include(CTest)
include(FindPkgConfig)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS_DEBUG_INIT "-Wall -Wextra -Werror -g3  -fsanitize=leak")
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Wall -O3")

find_package(spdlog REQUIRED)
find_package(cpprestsdk REQUIRED)


file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/lib/include)

include_directories(
    "${PROJECT_BINARY_DIR}"
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src

    SYSTEM
    ${PROJECT_SOURCE_DIR}/lib/srsran
    ${PROJECT_SOURCE_DIR}/lib/srsran/lib/include
    ${PROJECT_BINARY_DIR}/lib/srsran/lib/include
    ${PROJECT_BINARY_DIR}/lib/include
    ${PROJECT_SOURCE_DIR}/lib/include
    ${LIB_LIMESUITE_INCLUDE_DIRS}
    ${SPDLOG_INCLUDEDIR}
    )

set(ENABLE_SRSUE OFF CACHE BOOL "Build srsUE application")
set(ENABLE_SRSENB OFF CACHE BOOL "Build srsENB application")
set(ENABLE_SRSEPC OFF CACHE BOOL "Build srsEPC application")
add_subdirectory(lib/srsran)

configure_file("include/Version.h.in" "Version.h")

link_directories(
    ${LIB_LIMESUITE_LIBRARIES}
    ${PROJECT_BINARY_DIR}/lib/srsran/lib/src/phy/
    )


set(CMAKE_CXX_CLANG_TIDY clang-tidy --format-style=google --checks=clang-diagnostic-*,clang-analyzer-*,-*,bugprone*,modernize*,performance*)

add_executable(modem src/main.cpp src/SdrReader.cpp src/Phy.cpp
  src/CasFrameProcessor.cpp src/MbsfnFrameProcessor.cpp src/Rrc.cpp
    src/Gw.cpp src/RestHandler.cpp src/MeasurementFileWriter.cpp)

target_link_libraries( modem
    LINK_PUBLIC
    spdlog::spdlog
    srsran_phy
    srsran_mac
    srsran_rlc
    srsran_pdcp
    rrc_asn1
    config++
    cpprestsdk::cpprest
    gps
    ssl
    crypto
    SoapySDR

    debug
    lsan
)


install(TARGETS modem)
install(FILES supporting_files/5gmag-rt-modem.service DESTINATION /usr/lib/systemd/system)
install(FILES supporting_files/5gmag-rt.conf DESTINATION /etc)
install(FILES supporting_files/5gmag-rt DESTINATION /etc/default)
install(FILES supporting_files/pre_modem.sh DESTINATION /usr/share/5gmag-rt
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  )
install(FILES supporting_files/modem_measurements DESTINATION /etc/logrotate.d)

SET(CPACK_GENERATOR "DEB")
SET(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "5G-MAG Reference Tools <reference-tools@5g-mag.com>")
SET(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_SOURCE_DIR}/debian/conffiles;${CMAKE_SOURCE_DIR}/debian/postinst;")

INCLUDE(CPack)
