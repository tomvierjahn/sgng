##------------------------------------------------------------------------------
## sgng -- Surface-Reconstructing Growing Neural Gas
##
## Developed for my PhD dissertation
##   "Online Surface Reconstruction From Unorganized Point Clouds
##    With Integrated Texture Mapping"
##   Visualization and Computer Graphics Research Group
##   Department of Computer Science
##   University of Muenster
##   Germany
## Copyright (c) 2015, Tom Vierjahn
##------------------------------------------------------------------------------
##                                License
##
## This library/program is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published
## by the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## If you are using this library/program in a project, work or publication,
## please cite [1].
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.
##------------------------------------------------------------------------------
##                                References
##
## [1] Tom Vierjahn, Klaus Hinrichs:
##     "Surface-Reconstructing Growing Neural Gas:
##      A method for online construction of textured triangle meshes".
##     In Computers & Graphics, 51:190–201, 2015.
##     Doi: 10.1016/j.cag.2015.05.016.
##------------------------------------------------------------------------------

################################################################################
### cmake
################################################################################
cmake_minimum_required(VERSION 2.8.4)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake-modules)

add_definitions(-DIGNORE_COLOR)
add_definitions(-DNO_TEXTURES)
add_definitions(-DNO_FRAMELIST)
add_definitions(-DNO_ITERATIVE_POINTCLOUDS)


################################################################################
### project
################################################################################
project(surface-reconstruction)

### use ../bin as common output directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${COMMON_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${COMMON_OUTPUT_DIRECTORY})



################################################################################
### platform specific architecture
################################################################################
if(APPLE)
  set(CMAKE_OSX_ARCHITECTURES "x86_64")
endif()



################################################################################
### testing
################################################################################
enable_testing()
set(TEST_EXECUTABLES "" CACHE INTERNAL "" FORCE)



################################################################################
### dependencies
################################################################################
message("COD ${COMMON_OUTPUT_DIRECTORY}")
find_package(common 16.12.15.00 REQUIRED PATHS ${COMMON_OUTPUT_DIRECTORY})
include_directories(${common_INCLUDE_DIRS})



################################################################################
### warning levels
################################################################################
include(${PROJECT_SOURCE_DIR}/cmake-modules/WarningLevels.cmake)
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O")
endif()



################################################################################
### subdirectories
################################################################################
add_subdirectory(surface-reconstruction)



################################################################################
### target
################################################################################
add_custom_target(run_all_tests
                  COMMAND ${CMAKE_CTEST_COMMAND}
                  DEPENDS ${TEST_EXECUTABLES})



################################################################################
### copy dependencies
################################################################################
if (APPLE)
add_custom_command(
  TARGET run_all_tests POST_BUILD
  COMMAND cp ARGS -f -R ${common_LIBRARIES} $<CONFIGURATION> > /dev/null
  WORKING_DIRECTORY ${COMMON_OUTPUT_DIRECTORY}
  COMMENT "Copying common dylib." )
add_custom_command(
  TARGET run_all_tests POST_BUILD
  COMMAND cp ARGS -f -R ${common_DEBUG_LIBRARIES} $<CONFIGURATION> > /dev/null
  WORKING_DIRECTORY ${COMMON_OUTPUT_DIRECTORY}
  COMMENT "Copying common dylib." )
endif()

