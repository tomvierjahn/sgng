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
### sources
################################################################################
file(GLOB TESTS *.cc *.h)
file(GLOB EXTERNAL_SOURCES ../src/program_options.cc ../src/triangle_mesh.cc)
  
include_directories(..)
#include_directories(../../common/include)
include_directories(../../surface-reconstruction/src)



################################################################################
### dependencies
################################################################################

### gtest
if (NOT TARGET gtest)
  add_subdirectory(../../gtest ${CMAKE_CURRENT_BINARY_DIR}/gtest)
endif()
include_directories(../../gtest/include)


### boost
#set(BOOST_ROOT /usr/local/Cellar/boost/1.52.0)
set(Boost_NO_SYSTEM_PATHS ON)
find_package(Boost 1.55 EXACT REQUIRED date_time program_options system)
include_directories(${Boost_INCLUDE_DIRS})
if (MSVC)
  # Do not link Boost libraries automatically, since we explicitly link in CMake.
  add_definitions(-DBOOST_ALL_NO_LIB)
endif()




################################################################################
### additional source groups
################################################################################
source_group("External" FILES ${EXTERNAL_SOURCES})  




################################################################################
### architecture
################################################################################
if(APPLE)
  SET (CMAKE_OSX_ARCHITECTURES "x86_64")
endif()




################################################################################
### executable
################################################################################
add_executable(surface_reconstruction_test 
  ${TESTS} 
  ${EXTERNAL_SOURCES}
)

target_link_libraries(surface_reconstruction_test
  debug ${common_DEBUG_LIBRARIES} optimized ${common_LIBRARIES}
  ${Boost_LIBRARIES}
  gtest 
  gtest_main  
)

add_test(surface_reconstruction_test 
  ${COMMON_OUTPUT_DIRECTORY}/surface_reconstruction_test
)

set(TEST_EXECUTABLES 
  ${TEST_EXECUTABLES} 
  surface_reconstruction_test CACHE INTERNAL "" FORCE
)




################################################################################
### copy dependencies
################################################################################
add_custom_command(
  TARGET surface-surface_reconstruction_test POST_BUILD
  COMMAND cp ARGS -f -R ${common_LIBRARIES} $<CONFIGURATION> > /dev/null
  WORKING_DIRECTORY ${COMMON_OUTPUT_DIRECTORY}
  COMMENT "Copying common dylib." )
add_custom_command(
  TARGET surface-surface_reconstruction_test POST_BUILD
  COMMAND cp ARGS -f -R ${common_DEBUG_LIBRARIES} $<CONFIGURATION> > /dev/null
  WORKING_DIRECTORY ${COMMON_OUTPUT_DIRECTORY}
  COMMENT "Copying common dylib." )
