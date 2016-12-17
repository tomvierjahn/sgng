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
file(GLOB SURFACE_RECONSTRUCTION_SOURCES ./src/*.cc)
file(GLOB SURFACE_RECONSTRUCTION_HEADERS ./src/*.h)
file(GLOB SURFACE_RECONSTRUCTION_IO_SOURCES ./src/io/*.cc)
file(GLOB SURFACE_RECONSTRUCTION_IO_HEADERS ./src/io/*.h)



################################################################################
### dependencies
################################################################################

### Boost ###
#set(BOOST_ROOT /usr/local/Cellar/boost/1.52.0)
set(Boost_NO_SYSTEM_PATHS ON)
find_package(Boost 1.55 EXACT REQUIRED program_options date_time filesystem system thread)
include_directories(${Boost_INCLUDE_DIRS})
if(MSVC)
  # Do not link Boost libraries automatically, since we explicitly link in CMake.
  add_definitions(-DBOOST_ALL_NO_LIB)
  # Use shared boost libraries.
  add_definitions(-DBOOST_ALL_DYN_LINK)
endif(MSVC)


### TR1 implementation (platform dependent) ###
#if(NOT APPLE)
  # Use the Boost TR1 implementation.
  include_directories(${Boost_INCLUDE_DIRS}/boost/tr1)
#endif()


### ply ###
file(GLOB PLY_SOURCES thirdparty/ply-0.1/ply/*.cpp)
file(GLOB PLY_HEADERS thirdparty/ply-0.1/ply/*.hpp)
list(REMOVE_ITEM PLY_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ply-0.1/ply/config.hpp)
source_group("thirdparty\\ply" FILES ${PLY_HEADERS} ${PLY_SOURCES})
include_directories(./thirdparty/ply-0.1)

### io ###
find_package(io 16.12.15.00 REQUIRED PATHS ${COMMON_OUTPUT_DIRECTORY})
include_directories(${io_INCLUDE_DIRS})

## Eigen3 ##
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(
  ../../eigen3
  /usr/local/Cellar/Eigen/3.2.0/include/eigen3
  /usr/include/eigen3
  
)




################################################################################
### config file
################################################################################
if(MSVC)
  configure_file(./surface-reconstruction.cfg.in ${CMAKE_CURRENT_BINARY_DIR}/surface-reconstruction.cfg)
endif()
set(CONFIGURATION_SUBDIRS ${CMAKE_CONFIGURATION_TYPES})
foreach(CONFIGURATION_SUBDIR ${CONFIGURATION_SUBDIRS})
  configure_file(./surface-reconstruction.cfg.in ${COMMON_OUTPUT_DIRECTORY}/${CONFIGURATION_SUBDIR}/surface-reconstruction.cfg)
endforeach()

set(SURFACE_RECONSTRUCTION_USER_CONFIG_FILE "${COMMON_OUTPUT_DIRECTORY}/surface-reconstruction-user.cfg")
if(NOT EXISTS ${SURFACE_RECONSTRUCTION_USER_CONFIG_FILE})
  file(WRITE ${SURFACE_RECONSTRUCTION_USER_CONFIG_FILE} "# Add your configuration options here.
# Run 'surface-reconstruction --help' or look into program_options.cc for a list of possible options.
# Please note that you cannot overwrite options set on the command line or in surface-reconstruction.cfg.\n\n")
endif()



################################################################################
### additional source groups
################################################################################
source_group("io\\Source Files" FILES ${SURFACE_RECONSTRUCTION_IO_SOURCES})
source_group("io\\Header Files" FILES ${SURFACE_RECONSTRUCTION_IO_HEADERS})
source_group("config" FILES ${SURFACE_RECONSTRUCTION_USER_CONFIG_FILE})



################################################################################
### tests
################################################################################
add_subdirectory(test)



################################################################################
### additional include dirs
################################################################################
include_directories("./src" "./src/io")



################################################################################
### architecture
################################################################################
if(APPLE)
  SET (CMAKE_OSX_ARCHITECTURES "x86_64")
endif()



################################################################################
### executable
################################################################################
add_executable(surface-reconstruction
  ${SURFACE_RECONSTRUCTION_SOURCES}
  ${SURFACE_RECONSTRUCTION_HEADERS}
  ${SURFACE_RECONSTRUCTION_IO_SOURCES}
  ${SURFACE_RECONSTRUCTION_IO_HEADERS}
  ${SURFACE_RECONSTRUCTION_SHADERS}
  ${SURFACE_RECONSTRUCTION_USER_CONFIG_FILE}
  ${PLY_SOURCES}
  ${PLY_HEADERS}
)

### link ###
target_link_libraries(surface-reconstruction
  debug ${common_DEBUG_LIBRARIES} optimized ${common_LIBRARIES}
  debug ${io_DEBUG_LIBRARIES} optimized ${io_LIBRARIES}
  ${Boost_LIBRARIES}
)



################################################################################
### copy dependencies
################################################################################
if (APPLE)
add_custom_command(
  TARGET surface-reconstruction POST_BUILD
  COMMAND cp ARGS -f -R ${common_LIBRARIES} $<CONFIGURATION> > /dev/null
  WORKING_DIRECTORY ${COMMON_OUTPUT_DIRECTORY}
  COMMENT "Copying common dylib." )
add_custom_command(
  TARGET surface-reconstruction POST_BUILD
  COMMAND cp ARGS -f -R ${common_DEBUG_LIBRARIES} $<CONFIGURATION> > /dev/null
  WORKING_DIRECTORY ${COMMON_OUTPUT_DIRECTORY}
  COMMENT "Copying common dylib." )
add_custom_command(
  TARGET surface-reconstruction POST_BUILD
  COMMAND cp ARGS -f -R ${io_LIBRARIES} $<CONFIGURATION> > /dev/null
  WORKING_DIRECTORY ${COMMON_OUTPUT_DIRECTORY}
  COMMENT "Copying io dylib." )
add_custom_command(
  TARGET surface-reconstruction POST_BUILD
  COMMAND cp ARGS -f -R ${io_DEBUG_LIBRARIES} $<CONFIGURATION> > /dev/null
  WORKING_DIRECTORY ${COMMON_OUTPUT_DIRECTORY}
  COMMENT "Copying io dylib." )
endif()
