cmake_minimum_required(VERSION 3.16)

project (rp VERSION 1.0.1)

include(CTest)
include(FindPkgConfig)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS_DEBUG_INIT "-Wall -Wextra -Werror -g3")
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Wall -O3")

find_package(LimeSuite REQUIRED)
find_package(spdlog REQUIRED)
find_package(cpprestsdk REQUIRED)

set(ENABLE_SRSUE OFF CACHE BOOL "Build srsUE application")
set(ENABLE_SRSENB OFF CACHE BOOL "Build srsENB application")
set(ENABLE_SRSEPC OFF CACHE BOOL "Build srsEPC application")
add_subdirectory(lib/srslte)

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/lib/include)

include_directories(
    "${PROJECT_BINARY_DIR}"
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src

    SYSTEM
    ${PROJECT_SOURCE_DIR}/lib/srslte/lib/include
    ${PROJECT_BINARY_DIR}/lib/srslte/lib/include
    ${PROJECT_BINARY_DIR}/lib/include
    ${PROJECT_SOURCE_DIR}/lib/include
    ${LIB_LIMESUITE_INCLUDE_DIRS}
    ${SPDLOG_INCLUDEDIR}
    )

configure_file("include/Version.h.in" "Version.h")

link_directories(
    ${LIB_LIMESUITE_LIBRARIES}
    ${PROJECT_BINARY_DIR}/lib/srslte/lib/src/phy/
    )


set(CMAKE_CXX_CLANG_TIDY clang-tidy --format-style=google --checks=clang-diagnostic-*,clang-analyzer-*,-*,bugprone*,modernize*,performance*)

add_executable(rp src/main.cpp src/LimeSdrReader.cpp src/Phy.cpp
  src/CasFrameProcessor.cpp src/MbsfnFrameProcessor.cpp src/Rrc.cpp
    src/Gw.cpp src/RestHandler.cpp src/MeasurementFileWriter.cpp)
add_test(rp_test rp --config=./ofr-lrp.conf)

target_link_libraries( rp
    LINK_PUBLIC
    spdlog::spdlog
    srslte_phy
    srslte_mac
    srslte_upper
    rrc_asn1
    LimeSuite
    config++
    cpprestsdk::cpprest
    gps
    ssl
    crypto
)


install(TARGETS rp DESTINATION bin LIBRARY DESTINATION lib)
install(FILES supporting_files/rp.service DESTINATION /usr/lib/systemd/system)
install(FILES supporting_files/rp.conf DESTINATION /etc)
install(FILES supporting_files/obeca DESTINATION /etc/default)
install(FILES supporting_files/pre_rp.sh DESTINATION /usr/share/obeca
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  )
install(FILES supporting_files/rp_measurements DESTINATION /etc/logrotate.d)

SET(CPACK_GENERATOR "DEB")
SET(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Austrian Broadcasting Services <obeca@ors.at>")
SET(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_SOURCE_DIR}/debian/conffiles;${CMAKE_SOURCE_DIR}/debian/postinst;${CMAKE_SOURCE_DIR}/debian/rp.service;")

INCLUDE(CPack)
