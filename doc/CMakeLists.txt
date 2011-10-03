# Copyright 2011, Sebastien Dalibard, CNRS
#
# This file is part of hpp-constrained.
# hpp-constrained is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# hpp-constrained is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-constrained. If not, see <http://www.gnu.org/licenses/>.

SET(${${PROJECT_NAME}_SOURCE_DIR}/doc/header.html
    ${${PROJECT_NAME}_SOURCE_DIR}/doc/footer.html
    ${${PROJECT_NAME}_SOURCE_DIR}/doc/package.css
    ${${PROJECT_NAME}_SOURCE_DIR}/doc/package.dox
    )

# Configure package.dox file
CONFIGURE_FILE(${${PROJECT_NAME}_SOURCE_DIR}/doc/package.dox.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/package.dox
)

ADD_CUSTOM_COMMAND(OUTPUT ${${PROJECT_NAME}_SOURCE_DIR}/doc/doxygen-html/index.html
                   COMMAND ${DOXYGEN_EXECUTABLE} \"${CMAKE_CURRENT_BINARY_DIR}/package.dox\"
                   DEPENDS ${docfiles}
)

INSTALL(DIRECTORY ${${PROJECT_NAME}_BINARY_DIR}/doc/doxygen-html
        DESTINATION share/doc/hpp/gik/tutorial/)

# doxytag
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.doxytag
  DESTINATION share/doc/hpp/constrained/doxygen-html)