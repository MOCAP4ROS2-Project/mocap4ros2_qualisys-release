cmake_minimum_required(VERSION 3.10)
project(mocap4r2_qualisys_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(mocap4r2_control REQUIRED)
find_package(mocap4r2_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# Qualisys C++ SDK (MIT, unmodified sources), static and internal.
set(QSDK ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/qualisys_cpp_sdk)
add_library(qualisys_rt STATIC
  ${QSDK}/Deserializer.cpp
  ${QSDK}/Network.cpp
  ${QSDK}/RTPacket.cpp
  ${QSDK}/RTProtocol.cpp
  ${QSDK}/Serializer.cpp
  ${QSDK}/Settings.cpp
  ${QSDK}/SettingsDeserializer.cpp
  ${QSDK}/SettingsSerializer.cpp
  ${QSDK}/External/tinyxml2/tinyxml2.cpp
)
target_include_directories(qualisys_rt SYSTEM PUBLIC ${QSDK} ${QSDK}/External/tinyxml2)
target_compile_options(qualisys_rt PRIVATE -w)

add_library(${PROJECT_NAME} SHARED
  src/sdk_qualisys_source.cpp
  src/qualisys_conversion.cpp
  src/qualisys_driver.cpp
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  mocap4r2_control::mocap4r2_control
  ${mocap4r2_interfaces_TARGETS}
  rclcpp::rclcpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE qualisys_rt rclcpp_components::component)
rclcpp_components_register_nodes(${PROJECT_NAME} "mocap4r2_qualisys_driver::QualisysDriver")

add_executable(qualisys_driver src/qualisys_driver_main.cpp)
target_link_libraries(qualisys_driver ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES ThirdParty/qualisys_cpp_sdk/LICENSE.md
  DESTINATION share/${PROJECT_NAME} RENAME LICENSE.qualisys_cpp_sdk)
install(FILES ThirdParty/qualisys_cpp_sdk/External/tinyxml2/LICENSE.txt
  DESTINATION share/${PROJECT_NAME} RENAME LICENSE.tinyxml2)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS qualisys_driver RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # Linters on our code only: the Qualisys SDK is third-party code.
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  set(lint_paths include src test)
  ament_copyright(${lint_paths})
  ament_cppcheck(${lint_paths} LANGUAGE c++)
  ament_cpplint(${lint_paths})
  ament_lint_cmake()
  if(NOT "$ENV{ROS_DISTRO}" STREQUAL "humble")
    # uncrustify 0.72 of Humble formats differently: style is checked on newer distros
    ament_uncrustify(${lint_paths} LANGUAGE c++)
  endif()
  ament_xmllint()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_qualisys_conversion test/test_qualisys_conversion.cpp)
  target_link_libraries(test_qualisys_conversion ${PROJECT_NAME})
  ament_add_gtest(test_qualisys_driver test/test_qualisys_driver.cpp TIMEOUT 120)
  target_link_libraries(test_qualisys_driver ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mocap4r2_control mocap4r2_interfaces rclcpp)
ament_package()
