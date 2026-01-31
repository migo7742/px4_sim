#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "path_searching::path_searching" for configuration "Release"
set_property(TARGET path_searching::path_searching APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(path_searching::path_searching PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpath_searching.so"
  IMPORTED_SONAME_RELEASE "libpath_searching.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS path_searching::path_searching )
list(APPEND _IMPORT_CHECK_FILES_FOR_path_searching::path_searching "${_IMPORT_PREFIX}/lib/libpath_searching.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
