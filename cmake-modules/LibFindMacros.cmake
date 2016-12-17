# Works the same as find_package, but forwards the "REQUIRED" and "QUIET" arguments
# used for the current package. For this to work, the first parameter must be the
# prefix of the current package, then the prefix of the new package etc, which are
# passed to find_package.
macro (libfind_package PREFIX)
  set (LIBFIND_PACKAGE_ARGS ${ARGN})
  if (${PREFIX}_FIND_QUIETLY)
    set (LIBFIND_PACKAGE_ARGS ${LIBFIND_PACKAGE_ARGS} QUIET)
  endif (${PREFIX}_FIND_QUIETLY)
  if (${PREFIX}_FIND_REQUIRED)
    set (LIBFIND_PACKAGE_ARGS ${LIBFIND_PACKAGE_ARGS} REQUIRED)
  endif (${PREFIX}_FIND_REQUIRED)
  find_package(${LIBFIND_PACKAGE_ARGS})
endmacro (libfind_package)

# CMake developers made the UsePkgConfig system deprecated in the same release (2.6)
# where they added pkg_check_modules. Consequently I need to support both in my scripts
# to avoid those deprecated warnings. Here's a helper that does just that.
# Works identically to pkg_check_modules, except that no checks are needed prior to use.
macro (libfind_pkg_check_modules PREFIX PKGNAME)
  if (${CMAKE_MAJOR_VERSION} EQUAL 2 AND ${CMAKE_MINOR_VERSION} EQUAL 4)
    include(UsePkgConfig)
    pkgconfig(${PKGNAME} ${PREFIX}_INCLUDE_DIRS ${PREFIX}_LIBRARY_DIRS ${PREFIX}_LDFLAGS ${PREFIX}_CFLAGS)
  elseif (${CMAKE_MAJOR_VERSION} EQUAL 2 AND ${CMAKE_MINOR_VERSION} GREATER 7)
    find_package(PkgConfig)
    if (PKG_CONFIG_FOUND)
      pkg_check_modules(${PREFIX} ${PKGNAME} QUIET)
    endif (PKG_CONFIG_FOUND)
  else (${CMAKE_MAJOR_VERSION} EQUAL 2 AND ${CMAKE_MINOR_VERSION} EQUAL 4)
    find_package(PkgConfig)
    if (PKG_CONFIG_FOUND)
      pkg_check_modules(${PREFIX} ${PKGNAME})
    endif (PKG_CONFIG_FOUND)
  endif (${CMAKE_MAJOR_VERSION} EQUAL 2 AND ${CMAKE_MINOR_VERSION} EQUAL 4)
endmacro (libfind_pkg_check_modules)

# Combine a debug and a release library path into one string suitable for use with
# libfind_process. You only need to supply the library basename (say, Foo) and it
# will use the values of Foo_DEBUG_LIBRARY and Foo_RELEASE_LIBRARY (to be set before
# using find_library) to produce Foo_LIBRARY.
macro (libfind_join_debug_and_release_lib LIBRARY_BASENAME)
  if(${LIBRARY_BASENAME}_DEBUG_LIBRARY AND ${LIBRARY_BASENAME}_RELEASE_LIBRARY)
    set(${LIBRARY_BASENAME}_LIBRARY "debug;${${LIBRARY_BASENAME}_DEBUG_LIBRARY};optimized;${${LIBRARY_BASENAME}_RELEASE_LIBRARY}")
  else()
    set(${LIBRARY_BASENAME}_LIBRARY "${LIBRARY_BASENAME}_LIBRARY-NOTFOUND")
  endif()
endmacro(libfind_join_debug_and_release_lib)

# Do the final processing once the paths have been detected.
# If include dirs are needed, ${PREFIX}_PROCESS_INCLUDES should be set to contain
# all the variables, each of which contain one include directory.
# Ditto for ${PREFIX}_PROCESS_LIBS and library files.
# Will set ${PREFIX}_FOUND, ${PREFIX}_INCLUDE_DIRS and ${PREFIX}_LIBRARIES.
# Also handles errors in case library detection was required, etc.
macro (libfind_process PREFIX)
  # Skip processing if already processed during this run
  if (NOT ${PREFIX}_FOUND)
    # Start with the assumption that the library was found
    set (${PREFIX}_FOUND TRUE)

    # Process all includes and set _FOUND to false if any are missing
    foreach (i ${${PREFIX}_PROCESS_INCLUDES})
      if (${i})
        set (${PREFIX}_INCLUDE_DIRS ${${PREFIX}_INCLUDE_DIRS} ${${i}})
        mark_as_advanced(${i})
      else (${i})
        set (${PREFIX}_FOUND FALSE)
      endif (${i})
    endforeach (i)

    # Process all libraries and set _FOUND to false if any are missing
    foreach (i ${${PREFIX}_PROCESS_LIBS})
      if (${i})
        set (${PREFIX}_LIBRARIES ${${PREFIX}_LIBRARIES} ${${i}})
        mark_as_advanced(${i})
      else (${i})
        set (${PREFIX}_FOUND FALSE)
      endif (${i})
    endforeach (i)

    # Print message and/or exit on fatal error
    if (${PREFIX}_FOUND)
      if (NOT ${PREFIX}_FIND_QUIETLY)
        message (STATUS "Found ${PREFIX} ${${PREFIX}_VERSION}")
        mark_as_advanced(${PREFIX}_LIBRARIES)
        mark_as_advanced(${PREFIX}_INCLUDE_DIRS)
      endif (NOT ${PREFIX}_FIND_QUIETLY)
    else (${PREFIX}_FOUND)
      if (${PREFIX}_FIND_REQUIRED)
        foreach (i ${${PREFIX}_PROCESS_INCLUDES} ${${PREFIX}_PROCESS_LIBS})
          message("${i}=${${i}}")
        endforeach (i)
        message (FATAL_ERROR "Required library ${PREFIX} NOT FOUND.\nInstall the library (dev version) and try again. If the library is already installed, use ccmake to set the missing variables manually.")
      endif (${PREFIX}_FIND_REQUIRED)
    endif (${PREFIX}_FOUND)
  endif (NOT ${PREFIX}_FOUND)
endmacro (libfind_process)

macro(libfind_library PREFIX basename)
  set(TMP "")
  if(MSVC80)
    set(TMP -vc80)
  endif(MSVC80)
  if(MSVC90)
    set(TMP -vc90)
  endif(MSVC90)
  set(${PREFIX}_LIBNAMES ${basename}${TMP})
  if(${ARGC} GREATER 2)
    set(${PREFIX}_LIBNAMES ${basename}${TMP}-${ARGV2})
    string(REGEX REPLACE "\\." "_" TMP ${${PREFIX}_LIBNAMES})
    set(${PREFIX}_LIBNAMES ${${PREFIX}_LIBNAMES} ${TMP})
  endif(${ARGC} GREATER 2)
  find_library(${PREFIX}_LIBRARY
    NAMES ${${PREFIX}_LIBNAMES}
    PATHS ${${PREFIX}_PKGCONF_LIBRARY_DIRS}
  )
endmacro(libfind_library)

