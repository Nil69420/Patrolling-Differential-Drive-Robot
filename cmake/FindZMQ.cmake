# Copyright 2026 Nil
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

find_package(PkgConfig QUIET)
pkg_check_modules(PC_ZMQ QUIET libzmq)

find_path(ZMQ_INCLUDE_DIR
  NAMES zmq.h
  HINTS ${PC_ZMQ_INCLUDE_DIRS}
)

find_library(ZMQ_LIBRARY
  NAMES zmq
  HINTS ${PC_ZMQ_LIBRARY_DIRS}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZMQ
  REQUIRED_VARS ZMQ_LIBRARY ZMQ_INCLUDE_DIR
)

if(ZMQ_FOUND)
  set(ZMQ_LIBRARIES ${ZMQ_LIBRARY})
  set(ZMQ_INCLUDE_DIRS ${ZMQ_INCLUDE_DIR})
endif()

mark_as_advanced(ZMQ_INCLUDE_DIR ZMQ_LIBRARY)
