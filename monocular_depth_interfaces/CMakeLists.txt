cmake_minimum_required(VERSION 3.8)
project(monocular_depth_interfaces)

# ───────────────── compiler flags (unchanged) ────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─────────────────────── dependencies we need ────────────────────────────────
find_package(ament_cmake            REQUIRED)
find_package(std_msgs               REQUIRED)
find_package(rosidl_default_generators REQUIRED)   # ← adds the ros-IDL generators

# ───────────────────── generate the message code ─────────────────────────────
#   This call produces C++ + Python code for DepthStamped.msg
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DepthStamped.msg"
  DEPENDENCIES std_msgs
)

# ───────────────────── export runtime dependencies ───────────────────────────
ament_export_dependencies(rosidl_default_runtime)

# ───────────────────── tests (template left as-is) ───────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
