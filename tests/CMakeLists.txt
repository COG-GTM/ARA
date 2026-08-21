# Standalone unit/protocol test build for skydio_me_node.
#
# The production node links against the proprietary MPMS SDK (MMS,
# UDPCommInterface); these tests replace the small SDK surface used by the
# behavior code with header stubs under tests/stubs/ so the suite runs
# without the SDK. Build and run:
#
#   cmake -S tests -B build-tests
#   cmake --build build-tests -j
#   ctest --test-dir build-tests --output-on-failure

cmake_minimum_required(VERSION 3.14)
project(skydio_me_node_tests)

# Match the production ME build (C++14); tests must not rely on newer features.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

find_package(GTest REQUIRED)
enable_testing()

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

add_executable(skydio_me_tests
  ${CMAKE_CURRENT_SOURCE_DIR}/test_utility.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_mavlink_client.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test_traverse_to.cpp
  ${REPO_ROOT}/src/utility.cpp
  ${REPO_ROOT}/src/UTM.cpp
  ${REPO_ROOT}/src/SkydioMavlinkClient.cpp
  ${REPO_ROOT}/src/TraverseTo_impl.cpp
)

target_include_directories(skydio_me_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}         # FakeVehicle.h
  ${CMAKE_CURRENT_SOURCE_DIR}/stubs   # MMSTypes/ and MMSLib/ stand-ins
  ${REPO_ROOT}/include
  ${REPO_ROOT}/gen_include
)

target_link_libraries(skydio_me_tests
  GTest::gtest_main
  gdal
  pthread
)

include(GoogleTest)
gtest_discover_tests(skydio_me_tests PROPERTIES TIMEOUT 120)
