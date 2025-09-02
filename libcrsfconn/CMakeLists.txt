cmake_minimum_required(VERSION 3.8)
project(libcrsfconn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ASIO REQUIRED)


include_directories(
  include
  ${ASIO_INCLUDE_DIRS}
)

add_library(crsf SHARED
  src/crsf.cpp
  src/serial_transport.cpp
  src/messages.cpp
)

add_executable(crsf_example
  examples/print_rc_channels.cpp
)

target_link_libraries(crsf_example 
  crsf
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  set(ament_cmake_clang_format_CONFIG_FILE .clang-format)
  ament_clang_format()
  # skip copyright/cpplint checks as you already have
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  # ---- Add your test here ----
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(crsf_parser_test
    tests/crsf_parser_test.cpp
  )
  if(TARGET crsf_parser_test)
    target_link_libraries(crsf_parser_test crsf)  # link your library if needed
  endif()
  ament_add_gtest(msg_test
    tests/frame_to_msg.cpp
  )
  if(TARGET msg_test)
    target_link_libraries(msg_test crsf)  # link your library if needed
  endif()
endif()


install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  crsf_example
  DESTINATION lib/${PROJECT_NAME})


install(
  TARGETS crsf
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(crsf)

ament_package()
