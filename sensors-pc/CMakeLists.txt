cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(sensors-pc LANGUAGES CXX)

add_executable(sensors-decode
  ${CMAKE_CURRENT_SOURCE_DIR}/sensors-decode/main.cpp
)

target_link_libraries(sensors-decode
  sensors
  options::modern-cpp
)

add_executable(sensors-from-dior
  ${CMAKE_CURRENT_SOURCE_DIR}/sensors-from-dior/main.cpp
)

target_link_libraries(sensors-from-dior
  sensors
  options::modern-cpp
  depends::opencv
)

add_executable(sensors-preview
  ${CMAKE_CURRENT_SOURCE_DIR}/sensors-preview/main.cpp
)

target_link_libraries(sensors-preview
  sensors
  options::modern-cpp
  depends::opencv
)

add_executable(sensors-sort
  ${CMAKE_CURRENT_SOURCE_DIR}/sensors-sort/main.cpp
)

target_link_libraries(sensors-sort
  sensors
  options::modern-cpp
  depends::opencv
)
