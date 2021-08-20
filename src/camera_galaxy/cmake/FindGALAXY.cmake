# Copyright 2019 Zhushi Tech, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#[=======================================================================[.rst:
FindGALAXY
-------

Finds the GALAXY library.

Imported Targets
^^^^^^^^^^^^^^^^

This module provides the following imported targets, if found:

``GALAXY::GALAXY``
  The GALAXY library

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``GALAXY_FOUND``
  True if the system has the GALAXY library.
``GALAXY_VERSION`` and ``GALAXY_VERSION_STRING``
  The version of the GALAXY library which was found.
``GALAXY_INCLUDE_DIRS``
  The GALAXY include directories.
``GALAXY_LIBRARIES``
  The GALAXY libraries.

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``GALAXY_INCLUDE_DIR``
  The directory containing GxIAPI.h.
``GALAXY_LIBRARY``
  The path to the GxIAPI library.

#]=======================================================================]
# Try to find the GALAXY library

# Find the include path which includes inc/GxIAPI.h
find_path(GALAXY_INCLUDE_DIR
  NAMES "GxIAPI.h"
  HINTS "/opt/galaxy/inc/"
  DOC "The directory containing GxIAPI.h."
)

# Find the specific libary under <prefix>/lib/[x64|x86]/
if(WIN32)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(CMAKE_LIBRARY_ARCHITECTURE x64)
  else()
    set(CMAKE_LIBRARY_ARCHITECTURE x86)
  endif()
endif()

# Find the specific libary
find_library(GALAXY_LIBRARY
  NAMES "gxiapi"
  HINTS "/opt/galaxy/lib/"
  DOC "The path to the GxIAPI library file."
)

unset(CMAKE_LIBRARY_ARCHITECTURE)

# Extract version information
file(STRINGS "${GALAXY_INCLUDE_DIR}/GxIAPI.h" GALAXY_VERSION REGEX "@Version")
string(REGEX MATCH "[0-9.]+$" GALAXY_VERSION "${GALAXY_VERSION}")

# Conventional find package operation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GALAXY
  REQUIRED_VARS
    GALAXY_LIBRARY
    GALAXY_INCLUDE_DIR
  VERSION_VAR GALAXY_VERSION
)

# Setup import target which can be utilized by target_link_libraries
if(GALAXY_FOUND AND NOT TARGET GALAXY::GALAXY)
  add_library(GALAXY::GALAXY SHARED IMPORTED)
  set_target_properties(GALAXY::GALAXY PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${GALAXY_INCLUDE_DIR}"
    IMPORTED_LOCATION "${GALAXY_LIBRARY}"
    IMPORTED_NO_SONAME TRUE
  )
endif()

# For systems depend on INCLUDE_DIRS and LIBRARIES
set(GALAXY_INCLUDE_DIRS ${GALAXY_INCLUDE_DIR})
set(GALAXY_LIBRARIES ${GALAXY_LIBRARY})

mark_as_advanced(
  GALAXY_INCLUDE_DIR
  GALAXY_LIBRARY
)

# compatibility variables
set(GALAXY_VERSION_STRING ${GALAXY_VERSION})
