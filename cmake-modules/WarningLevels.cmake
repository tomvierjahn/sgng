# Used to setup agreed upon warning levels
# It will define
# GCC_CUSTOM_WARNING_LEVEL 

### warning levels ###
if(MSVC)
  # Force to always compile with W4
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
  # Warning C4503 is about mangled symbols in boost that are truncated because they
  # are too long. Warning C4996 is about MS-specific "unsafe" C standard library functions.
  # We cannot do much about them, so we disable them.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4503 /wd4996")

elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  # Update if necessary
  set(GCC_CUSTOM_WARNING_LEVEL "")
  
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wall")
  ### the following are enabled by -Wall ###
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Waddress")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wchar-subscripts")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wcomment")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wformat")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wimplicit")
  ### included in -Wimplicit ###
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Werror-implicit-function-declaration")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Werror-implicit-int")
  ### END: included in -Wimplicit ###
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wmain")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wmissing-braces")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wnonull")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wreturn-type")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wsequence-point")
  ### these go together ###
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -fstrict-aliasing")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wstrict-aliasing")
  ### END: these go together ###
  ### these go together ###
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -fstrict-overflow")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wstrict-overflow")
  ### END: these go together ###
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wswitch")
  ### these go together ###
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -trigraphs")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wtrigraphs")
  ### END: these go together ###
  
  
  ### these might be default ###
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wdiv-by-zero")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wendif-labels")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wformat-extra-args")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Winvalid-offsetof")
 
  ### this needs -pedantic ###
  # this might interfere with boost::python or other
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wlong-long")
  # so better use
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wno-long-long")
  ### END: this needs -pedantic ###
 
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wmultichar")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Woverflow")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wno-system-headers")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wvariadic-macros")
  
  ### non-default flags ###
  # this interferes heavily with boost::python and stl vectors
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Waggregate-return")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wcast-align")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wcast-qual")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wconversion")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wdisabled-optimization")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wfloat-equal")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wformat=2")
  ### included in -Wformat=2 ###
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wformat-nonliteral")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wformat-security")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wformat-y2k")
  ### END: included in -Wformat=2 ###
  
  ### this seems to be not working with boost -- once again
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wmissing-noreturn")
  ### END:  this seems to be not working with boost -- once again
  
  
  ### learn about global constructors ###
  # this interferes heavily with boost::python
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wglobal-constructors")
  ### END: learn about global constructors ###
  
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wimport")
  ### these go together ###
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wuninitialized")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Winit-self")
  ### END: these go together ###
  
  ### learn about inline warnings ###
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Winline")
  ### END: learn about inline warnings ###
  
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Winvalid-pch")
  
  ### This might be useful for debugging ... Replace len by the num of bytes ###
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wlarger-than-len")
  ### END: This might be useful for debugging ... Replace len by the num of bytes ###
  ### these seem to go together. Learn about it ###
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -funsafe-loop-optimizations")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wunsafe-loop-optimizations")
  ### END: these seem to go together. Learn about it ###
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wmissing-format-attribute" )
  ### xcode seems to use include dirs that do not exist
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wmissing-include-dir")
  ### END: xcode seems to use include dirs that do not exist
  
  
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wmissing-prototypes")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wpacked")
  ### learn more about padding then fix it ###
  #set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wpadded")
  ### END: learn more about padding then fix it ###
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wparentheses")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wpointer-arith")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wredundant-decls")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wshadow")
  
  ### these go together ###
  ### again some boost libs do not like this
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -fstack-protector")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wstack-protector")
  ### END: these go together ###
  
  ### these go together ###
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -fstrict-aliasing")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wstrict-aliasing=2")
  ### END: these go together ###
  
  ### these go together ###
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -fstrict-overflow")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wstrict-overflow=1")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wstrict-overflow=2")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wstrict-overflow=3")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wstrict-overflow=4")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wstrict-overflow=5")
  ### END: these go together ###
  
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wswitch-default")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wswitch-enum")
  
  ### this might conflict with system headers. If it does use only -Wall. ###
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wunknown-pragmas")
  ### END: this might conflict with system headers. If it does use only -Wall. ###
  
  ### this doesn't work with the system headers ###
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wunreachable-code")
  ### END: this doesn't work with the system headers ###
  
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wunused")
  ### these are enabled by -Wunused ###
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wunused-function")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wunused-label")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wunused-parameter")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wunused-value")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wunused-variable")
  ### END: these are enabled by -Wunused ###
  
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wvolatile-register-var")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wwrite-strings")
  
  
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wextra")
  ### the following are included in -Wextra ###
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wmissing-field-initializers")
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Wsign-compare")
   
  
  # set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Werror")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -pedantic")
  set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -pedantic-errors")
  
  #set(GCC_CUSTOM_WARNING_LEVEL "${GCC_CUSTOM_WARNING_LEVEL} -Weffc++")
endif()