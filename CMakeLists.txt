cmake_minimum_required(VERSION 3.5)

project (modem VERSION 1.2.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include(CTest)
include(FindPkgConfig)

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
    )

set(ENABLE_SRSUE OFF CACHE BOOL "Build srsUE application")
set(ENABLE_SRSENB OFF CACHE BOOL "Build srsENB application")
set(ENABLE_SRSEPC OFF CACHE BOOL "Build srsEPC application")
add_subdirectory(lib/srsran)

find_package(cpprestsdk REQUIRED)
include_directories(
  SYSTEM
    ${SPDLOG_INCLUDEDIR}
    )

configure_file("include/Version.h.in" "Version.h")

link_directories(
    ${LIB_LIMESUITE_LIBRARIES}
    ${PROJECT_BINARY_DIR}/lib/srsran/lib/src/phy/
    )


# Only enable clang-tidy if it's actually installed - this was previously
# unconditional, making every compile fail with "Error running 'clang-tidy':
# no such file or directory" on any machine without it (this build was never
# attempted on such a machine before now).
#
# clang-tidy runs its own bundled Clang frontend against every file,
# separately from whatever compiler (CMAKE_CXX_COMPILER, GCC here) actually
# builds it. Clang's built-in heuristic for locating a GCC installation's own
# C++ headers can lag behind very recent GCC releases -- seen with GCC 15 +
# clang-tidy 21: files fail with "'string'/'cstddef' file not found" because
# clang-tidy's frontend can't locate libstdc++ at all, even though the real
# build (GCC) has no such problem. Fix: ask the ACTUAL configured compiler
# for its real system include search path and hand those to clang-tidy
# explicitly, rather than relying on its own (version-lagging) auto-detection.
find_program(CLANG_TIDY_EXE NAMES clang-tidy)
if(CLANG_TIDY_EXE)
  set(CLANG_TIDY_EXTRA_ARGS "")
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    execute_process(
      COMMAND ${CMAKE_CXX_COMPILER} -E -x c++ -Wp,-v /dev/null
      ERROR_VARIABLE _cxx_verbose_includes
      OUTPUT_QUIET)
    string(REGEX MATCH "#include <\\.\\.\\.> search starts here:\n(.*)\nEnd of search list"
           _matched "${_cxx_verbose_includes}")
    if(_matched)
      string(REPLACE "\n" ";" _include_lines "${CMAKE_MATCH_1}")
      foreach(_line ${_include_lines})
        string(STRIP "${_line}" _path)
        if(_path)
          list(APPEND CLANG_TIDY_EXTRA_ARGS "--extra-arg=-isystem${_path}")
        endif()
      endforeach()
    endif()
  endif()
  # -modernize-use-scoped-lock: clang-tidy 21 crashes on this file set, taking the build with it:
  #   UseScopedLockCheck::diagOnSingleLock(...) -> Segmentation fault
  # while analysing src/main.cpp. The check is disabled rather than the whole analysis, so
  # everything else clang-tidy reports is still enforced. Remove this once the crash is fixed
  # upstream. The same exclusion is in 5G-MAG/rt-mbms-modem PR #53, which hit it independently.
  set(CMAKE_CXX_CLANG_TIDY clang-tidy --format-style=google --checks=clang-diagnostic-*,clang-analyzer-*,-*,bugprone*,modernize*,-modernize-use-scoped-lock,performance* ${CLANG_TIDY_EXTRA_ARGS})
endif()

add_executable(modem src/main.cpp src/SdrReader.cpp src/Phy.cpp
  src/CasFrameProcessor.cpp src/MbsfnFrameProcessor.cpp src/Rrc.cpp
  src/Gw.cpp src/RestHandler.cpp src/MeasurementFileWriter.cpp src/MultichannelRingbuffer.cpp)

target_link_libraries( modem
    LINK_PUBLIC
    srsran_phy
    srsran_mac
    srsran_rlc
    srsran_pdcp
    srslog
    rrc_asn1
    config++
    cpprestsdk::cpprest
    gps
    ssl
    crypto
    SoapySDR
)


install(TARGETS modem)
install(FILES supporting_files/5gmag-rt-modem.service DESTINATION /usr/lib/systemd/system)
install(FILES supporting_files/rt-common-shared/mbms/common-config/5gmag-rt.conf DESTINATION /etc)
install(FILES supporting_files/rt-common-shared/mbms/common-config/5gmag-rt DESTINATION /etc/default)
install(FILES supporting_files/pre_modem.sh DESTINATION /usr/share/5gmag-rt
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  )
install(FILES supporting_files/modem_measurements DESTINATION /etc/logrotate.d)

SET(CPACK_GENERATOR "DEB")
SET(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "5G-MAG Reference Tools <reference-tools@5g-mag.com>")
SET(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_SOURCE_DIR}/debian/conffiles;${CMAKE_SOURCE_DIR}/debian/postinst;")

INCLUDE(CPack)
