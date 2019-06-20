#
# Copyright (c) 2017 TOYOTA MOTOR CORPORATION
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

include(FindPkgConfig)
pkg_check_modules(AFB REQUIRED afb-daemon)

# We do not want a prefix for our module
set(CMAKE_SHARED_MODULE_PREFIX "")

set(TARGETS_CARLA carla-client-service)

add_library(${TARGETS_CARLA} MODULE
   carlaclient.cpp
	cansender.cpp
	canencoder.cpp
   main.cpp
   )

target_include_directories(${TARGETS_CARLA}
    PRIVATE
        ${AFB_INCLUDE_DIRS}
        ../src)

target_link_libraries(${TARGETS_CARLA}
    PRIVATE
        ${AFB_LIBRARIES})

target_compile_definitions(${TARGETS_CARLA}
    PRIVATE
        AFB_BINDING_VERSION=3
        # We do not want source location of messages
        AFB_BINDING_PRAGMA_NO_VERBOSE_DETAILS
        WINMAN_VERSION_STRING="${PACKAGE_VERSION}"
        _GNU_SOURCE)

if(NOT ${CMAKE_BUILD_TYPE} STREQUAL "Release")
   target_compile_definitions(${TARGETS_CARLA}
       PRIVATE
           _GLIBCXX_DEBUG)
endif()

target_compile_options(${TARGETS_CARLA}
    PRIVATE
        -Wall -Wextra -Wno-unused-parameter -Wno-comment -Wno-missing-field-initializers)

set_target_properties(${TARGETS_CARLA}
    PROPERTIES
    # INTERPROCEDURAL_OPTIMIZATION ON

        CXX_EXTENSIONS OFF
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED ON

        C_EXTENSIONS OFF
        C_STANDARD 99
        C_STANDARD_REQUIRED ON)

if(NOT EXISTS ${PROJECT_BINARY_DIR}/package)
   add_custom_command(TARGET ${TARGETS_CARLA} POST_BUILD
      COMMAND cp -rf ${PROJECT_SOURCE_DIR}/package ${PROJECT_BINARY_DIR}
   )
endif()

add_custom_command(TARGET ${TARGETS_CARLA} POST_BUILD
   COMMAND mkdir -p ${PROJECT_BINARY_DIR}/package/root/lib
   COMMAND cp -rf ${PROJECT_BINARY_DIR}/src/${TARGETS_CARLA}.so ${PROJECT_BINARY_DIR}/package/root/lib
   COMMAND wgtpkg-pack -f -o ${PROJECT_BINARY_DIR}/package/${TARGETS_CARLA}.wgt ${PROJECT_BINARY_DIR}/package/root
)

add_custom_target(package DEPENDS ${PROJECT_BINARY_DIR}/package/root
   COMMAND wgtpkg-pack -f -o ${PROJECT_BINARY_DIR}/package/${TARGETS_CARLA}.wgt ${PROJECT_BINARY_DIR}/package/root
)
