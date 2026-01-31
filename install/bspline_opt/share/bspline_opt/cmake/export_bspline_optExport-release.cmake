#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "bspline_opt::bspline_opt" for configuration "Release"
set_property(TARGET bspline_opt::bspline_opt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(bspline_opt::bspline_opt PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libbspline_opt.so"
  IMPORTED_SONAME_RELEASE "libbspline_opt.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS bspline_opt::bspline_opt )
list(APPEND _IMPORT_CHECK_FILES_FOR_bspline_opt::bspline_opt "${_IMPORT_PREFIX}/lib/libbspline_opt.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
