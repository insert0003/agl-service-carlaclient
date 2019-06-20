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

cmake_minimum_required(VERSION 2.8)

project(CarlaClientTMC)

set(PACKAGE_VERSION_MAJOR 0)
set(PACKAGE_VERSION_MINOR 0)
set(PACKAGE_VERSION_REVISION 1)
set(PACKAGE_VERSION "${PACKAGE_VERSION_MAJOR}.${PACKAGE_VERSION_MINOR}.${PACKAGE_VERSION_REVISION}")

find_package(PkgConfig REQUIRED)
include(GNUInstallDirs)

set(SANITIZER_MODE "none" CACHE STRING "Build using a specific sanitizer (e.g. 'address', 'thread', 'leak', 'undefined'), depends on compiler; default none")

set(LINK_LIBCXX OFF CACHE BOOL "Link against LLVMs libc++")

add_subdirectory(src)
