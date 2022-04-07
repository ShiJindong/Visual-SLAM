#----------------------------------------------------------------
# Generated CMake target import file for configuration "“Release”".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "MyHello::hello" for configuration "“Release”"
set_property(TARGET MyHello::hello APPEND PROPERTY IMPORTED_CONFIGURATIONS “RELEASE”)
set_target_properties(MyHello::hello PROPERTIES
  IMPORTED_LOCATION_“RELEASE” "${_IMPORT_PREFIX}/lib/libhello.so"
  IMPORTED_SONAME_“RELEASE” "libhello.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS MyHello::hello )
list(APPEND _IMPORT_CHECK_FILES_FOR_MyHello::hello "${_IMPORT_PREFIX}/lib/libhello.so" )

# Import target "MyHello::sayhello" for configuration "“Release”"
set_property(TARGET MyHello::sayhello APPEND PROPERTY IMPORTED_CONFIGURATIONS “RELEASE”)
set_target_properties(MyHello::sayhello PROPERTIES
  IMPORTED_LOCATION_“RELEASE” "${_IMPORT_PREFIX}/bin/sayhello"
  )

list(APPEND _IMPORT_CHECK_TARGETS MyHello::sayhello )
list(APPEND _IMPORT_CHECK_FILES_FOR_MyHello::sayhello "${_IMPORT_PREFIX}/bin/sayhello" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
