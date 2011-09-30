# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)

SET(PROJECT_NAME hpp-constrained-planner)
SET(PROJECT_VERSION 0.1)
SET(PROJECT_DESCRIPTION "Insert package description here...")
SET(PROJECT_URL "Insert package webpage here...")

ADD_REQUIRED_DEPENDENCY(hpp-constrained >= 0.1)
ADD_REQUIRED_DEPENDENCY(hpp-core >= 0.1)

SET(${PROJECT_NAME}_HEADERS
  include/hpp-constrained-planner/fwd.hh
  include/hpp-constrained-planner/hpp-constrained-planner.hh
)

SETUP_PROJECT()

# Search for Boost.
# Boost.Test is used by the test suite.
SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREAD ON)
FIND_PACKAGE(Boost 1.40 COMPONENTS unit_test_framework REQUIRED)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)
SETUP_PROJECT_FINALIZE()
