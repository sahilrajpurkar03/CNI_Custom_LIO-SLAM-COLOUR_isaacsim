# Install script for directory: /home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/src/isaac_sim_pointcloud_tool

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/install/isaac_sim_pointcloud_tool")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/isaac_sim_pointcloud_tool/converter" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/isaac_sim_pointcloud_tool/converter")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/isaac_sim_pointcloud_tool/converter"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/isaac_sim_pointcloud_tool" TYPE EXECUTABLE FILES "/home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/build/isaac_sim_pointcloud_tool/converter")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/isaac_sim_pointcloud_tool/converter" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/isaac_sim_pointcloud_tool/converter")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/isaac_sim_pointcloud_tool/converter"
         OLD_RPATH "/opt/ros/humble/lib:/home/rajpurkar/miniconda3/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/isaac_sim_pointcloud_tool/converter")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/build/isaac_sim_pointcloud_tool/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/isaac_sim_pointcloud_tool")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/build/isaac_sim_pointcloud_tool/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/isaac_sim_pointcloud_tool")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/isaac_sim_pointcloud_tool/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/isaac_sim_pointcloud_tool/environment" TYPE FILE FILES "/home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/build/isaac_sim_pointcloud_tool/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/isaac_sim_pointcloud_tool/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/isaac_sim_pointcloud_tool/environment" TYPE FILE FILES "/home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/build/isaac_sim_pointcloud_tool/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/isaac_sim_pointcloud_tool" TYPE FILE FILES "/home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/build/isaac_sim_pointcloud_tool/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/isaac_sim_pointcloud_tool" TYPE FILE FILES "/home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/build/isaac_sim_pointcloud_tool/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/isaac_sim_pointcloud_tool" TYPE FILE FILES "/home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/build/isaac_sim_pointcloud_tool/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/isaac_sim_pointcloud_tool" TYPE FILE FILES "/home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/build/isaac_sim_pointcloud_tool/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/isaac_sim_pointcloud_tool" TYPE FILE FILES "/home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/build/isaac_sim_pointcloud_tool/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/build/isaac_sim_pointcloud_tool/ament_cmake_index/share/ament_index/resource_index/packages/isaac_sim_pointcloud_tool")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/isaac_sim_pointcloud_tool/cmake" TYPE FILE FILES
    "/home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/build/isaac_sim_pointcloud_tool/ament_cmake_core/isaac_sim_pointcloud_toolConfig.cmake"
    "/home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/build/isaac_sim_pointcloud_tool/ament_cmake_core/isaac_sim_pointcloud_toolConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/isaac_sim_pointcloud_tool" TYPE FILE FILES "/home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/src/isaac_sim_pointcloud_tool/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/rajpurkar/CNI_Custom_LIO-SLAM-COLOUR_isaacsim/ros2_ws/build/isaac_sim_pointcloud_tool/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
