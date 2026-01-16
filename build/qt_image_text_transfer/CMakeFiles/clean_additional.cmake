# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "CMakeFiles/qt_image_text_transfer_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/qt_image_text_transfer_autogen.dir/ParseCache.txt"
  "qt_image_text_transfer_autogen"
  )
endif()
