cmake_minimum_required(VERSION 3.14)

set(RDK_DIR            ${CMAKE_CURRENT_SOURCE_DIR})
set(RDK_3RD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party)
set(RDK_CMAKE_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(RDK_SDK_DIR       ${CMAKE_CURRENT_SOURCE_DIR}/sdk)
set(RDK_STRATA_LIBRARY strata_shared)

set(CMAKE_MODULE_PATH "${RDK_CMAKE_DIR};${CMAKE_MODULE_PATH}")
file(GLOB RDK_CMAKE_FILES ${RDK_CMAKE_DIR}/*.cmake)
add_custom_target(rdk SOURCES
    ${RDK_CMAKE_FILES}
    )

# set default values for C and C++ flags (flags can be overwritten by targets)
set(CMAKE_USER_MAKE_RULES_OVERRIDE     ${RDK_CMAKE_DIR}/c_flags_overrides.cmake)
set(CMAKE_USER_MAKE_RULES_OVERRIDE_CXX ${RDK_CMAKE_DIR}/cxx_flags_overrides.cmake)

# configure output directories
if(NOT RDK_BINARY_DIR)
    set(RDK_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/bin")
endif()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${RDK_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${RDK_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${RDK_BINARY_DIR}/lib")
set(CMAKE_PDB_OUTPUT_DIRECTORY "${RDK_BINARY_DIR}/.debug")

project(rdk)

# project specific settings
include(${RDK_CMAKE_DIR}/build_settings.cmake)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

option(CODE_COVERAGE "Enable code coverage for integration tests" OFF)
option(SANITIZE "Enable sanitizer" OFF)
option(ANALYZE "Enable analyze targets" OFF)
option(FORMAT "Enable format targets" OFF)

# Dependencies of installed (wrapped) shared libraries may not be resolved
# properly by dynamic linker on Linux and MacOS systems, unless Runtime search
# Path (RPATH) is set.
# Windows doesn't support the concept of RPATH.
# -----------------------------------------------------------------------------
# Specify relative RPATHs for binaries linked in the build tree (in other words -
# use directory of a loaded library as a search path for its dependencies)
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    # @loader_path (MacOS specific) resolves to a path of the directory
    # containing the dynamic library that is being loaded
    set(CMAKE_BUILD_RPATH "@loader_path;@loader_path/lib")
else()
    # $ORIGIN token works on platforms such as Linux.
    # This property has no effect on platforms that do not support
    # the $ORIGIN token in RPATH (such as MacOS)
    set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)
endif()

# get project's version
include(version)
add_definitions( -DSDK_VERSION="${SDK_VERSION}" )
add_definitions( -DSDK_VERSION_FULL="${SDK_VERSION_FULL}" )
add_definitions( -DSDK_COMMIT_HASH="${SDK_COMMIT_HASH}" )

include(TestBigEndian)

# build all targets using position independent code (PIC)
# It is important that this definition comes before any subdirectory is
# included or any target is defined such that all targets are compiled
# using position independent code.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# on MinGW don't add the prefix "lib" to shared and static libraries
if(MINGW)
   set(CMAKE_SHARED_LIBRARY_PREFIX "")
   set(CMAKE_STATIC_LIBRARY_PREFIX "")
endif()

test_big_endian(SDK_TARGET_PLATFORM_BIG_ENDIAN)
if(SDK_TARGET_PLATFORM_BIG_ENDIAN)
    add_definitions(-DSDK_TARGET_PLATFORM_BIG_ENDIAN)
endif()
include(macros)

add_subdirectory("${RDK_3RD_PARTY_DIR}/libs/")
add_subdirectory("./external")


# the following uses Strata cmake extensions (which are included in "external")

include(../local_settings.cmake OPTIONAL)


# SDK C library
add_subdirectory(sdk)

# examples
add_subdirectory(examples)

# tools
add_subdirectory(tools)
