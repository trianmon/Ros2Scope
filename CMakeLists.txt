# SPDX-FileCopyrightText: 2026 Georgy Grigoriev (GitHub: trianmon)
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(ros2scope)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)
include(GNUInstallDirs)

find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcpputils REQUIRED)
find_package(OpenGL REQUIRED)

if(NOT TARGET rclcpp::rclcpp)
	if(DEFINED ENV{ROS_DISTRO} AND NOT "$ENV{ROS_DISTRO}" STREQUAL "")
		set(_r2s_ros_distro "$ENV{ROS_DISTRO}")
	else()
		set(_r2s_ros_distro "<ros_distro>")
	endif()
	message(FATAL_ERROR "rclcpp target not found. Source ROS environment first: source /opt/ros/${_r2s_ros_distro}/setup.bash")
endif()

FetchContent_Declare(
	glfw_src
	GIT_REPOSITORY https://github.com/glfw/glfw.git
	GIT_TAG 3.4
)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(glfw_src)

FetchContent_Declare(
	imgui_src
	GIT_REPOSITORY https://github.com/ocornut/imgui.git
	GIT_TAG docking
)
FetchContent_MakeAvailable(imgui_src)

set(R2S_GLFW_TARGET glfw)

add_library(imgui STATIC
	${imgui_src_SOURCE_DIR}/imgui.cpp
	${imgui_src_SOURCE_DIR}/imgui_demo.cpp
	${imgui_src_SOURCE_DIR}/imgui_draw.cpp
	${imgui_src_SOURCE_DIR}/imgui_tables.cpp
	${imgui_src_SOURCE_DIR}/imgui_widgets.cpp
	${imgui_src_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
	${imgui_src_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
	${imgui_src_SOURCE_DIR}
	${imgui_src_SOURCE_DIR}/backends
)
target_link_libraries(imgui PUBLIC ${R2S_GLFW_TARGET} OpenGL::GL)

add_library(ros2scope_plugin_api INTERFACE)
target_include_directories(ros2scope_plugin_api INTERFACE core/plugin_api/include)

add_library(ros2scope_workspace_env STATIC
	core/workspace_env/src/workspace_environment.cpp
)
target_include_directories(ros2scope_workspace_env PUBLIC core/workspace_env/include)

add_library(ros2scope_layout STATIC
	core/layout/src/layout_manager.cpp
)
target_include_directories(ros2scope_layout PUBLIC core/layout/include)

add_library(ros2scope_type_introspection STATIC
	core/type_introspection/src/type_resolver.cpp
)
target_include_directories(ros2scope_type_introspection PUBLIC core/type_introspection/include)
target_link_libraries(ros2scope_type_introspection PUBLIC rcpputils::rcpputils)
target_link_libraries(ros2scope_type_introspection PUBLIC ros2scope_workspace_env)

add_library(ros2scope_ros_bridge STATIC
	core/ros_bridge/src/ros_bridge.cpp
)
target_include_directories(ros2scope_ros_bridge PUBLIC core/ros_bridge/include)
target_link_libraries(ros2scope_ros_bridge PUBLIC rclcpp::rclcpp)

if(TARGET rcl_interfaces::rcl_interfaces__rosidl_typesupport_cpp)
	target_link_libraries(ros2scope_ros_bridge PUBLIC rcl_interfaces::rcl_interfaces__rosidl_typesupport_cpp)
endif()

add_library(ros2scope_plugin_loader STATIC
	core/plugin_loader/src/plugin_loader.cpp
)
target_include_directories(ros2scope_plugin_loader PUBLIC core/plugin_loader/include)
target_link_libraries(ros2scope_plugin_loader PUBLIC ros2scope_plugin_api dl)

add_library(ros2scope_app STATIC
	core/app/src/application.cpp
)
target_include_directories(ros2scope_app PUBLIC
	core/app/include
	core/event_bus/include
)
target_link_libraries(ros2scope_app PUBLIC rclcpp::rclcpp)
target_link_libraries(ros2scope_app
	PUBLIC
		imgui
		ros2scope_layout
		ros2scope_plugin_api
		ros2scope_plugin_loader
		ros2scope_ros_bridge
		ros2scope_type_introspection
		ros2scope_workspace_env
)

add_executable(r2s src/main.cpp)
target_link_libraries(r2s PRIVATE ros2scope_app)

function(add_r2s_plugin target source_file)
	add_library(${target} MODULE ${source_file})
	target_link_libraries(${target} PRIVATE
		imgui
		rclcpp::rclcpp
		ros2scope_plugin_api
		ros2scope_ros_bridge
		ros2scope_type_introspection
	)
	target_include_directories(${target} PRIVATE
		core/plugin_api/include
		core/ros_bridge/include
		core/type_introspection/include
	)
	set_target_properties(${target} PROPERTIES PREFIX "")

	install(TARGETS ${target}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/plugins
	)
endfunction()

add_r2s_plugin(r2s_plugin_graph plugins/graph/src/graph_plugin.cpp)
add_r2s_plugin(r2s_plugin_topic_echo plugins/topic_echo/src/topic_echo_plugin.cpp)
add_r2s_plugin(r2s_plugin_plot plugins/plot/src/plot_plugin.cpp)
add_r2s_plugin(r2s_plugin_param_editor plugins/param_editor/src/param_editor_plugin.cpp)
add_r2s_plugin(r2s_plugin_image_viewer plugins/image_viewer/src/image_viewer_plugin.cpp)

install(TARGETS
	r2s
	ros2scope_workspace_env
	ros2scope_layout
	ros2scope_type_introspection
	ros2scope_ros_bridge
	ros2scope_plugin_loader
	ros2scope_app
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY
	core/app/include/
	core/event_bus/include/
	core/layout/include/
	core/plugin_api/include/
	core/plugin_loader/include/
	core/ros_bridge/include/
	core/type_introspection/include/
	core/workspace_env/include/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
