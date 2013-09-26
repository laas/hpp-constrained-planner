# Copyright 2011, Sebastien Dalibard, CNRS
#
# This file is part of hpp-constrained-planner.
# hpp-constrained-planner is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# hpp-constrained-planner is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-constrained-planner. If not, see <http://www.gnu.org/licenses/>.

INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}")
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}")
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}")

SET(LIBRARY_NAME ${PROJECT_NAME})

SET(${PACKAGE_NAME}_SOURCES
  planner.cc
  grasping-planner.cc
  )

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  ${${PACKAGE_NAME}_SOURCES}
  )

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-constrained)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)