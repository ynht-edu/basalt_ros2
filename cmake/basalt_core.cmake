include(FetchContent)

set(BASALT_MARCH native CACHE STRING "-march for basalt and everything linking it, empty to disable")

FetchContent_Declare(sophus
  GIT_REPOSITORY https://github.com/strasdat/Sophus.git
  GIT_TAG 1.24.6
  SOURCE_SUBDIR none)
FetchContent_Declare(magic_enum
  GIT_REPOSITORY https://github.com/Neargye/magic_enum.git
  GIT_TAG v0.9.7
  SOURCE_SUBDIR none)
FetchContent_Declare(basalt_headers
  GIT_REPOSITORY https://github.com/VladyslavUsenko/basalt-headers-mirror.git
  GIT_TAG b7f915b195c87c8373bf93f97a0710bcdaffd718
  SOURCE_SUBDIR none)
FetchContent_Declare(opengv
  GIT_REPOSITORY https://github.com/laurentkneip/opengv.git
  GIT_TAG 91f4b19c73450833a40e463ad3648aae80b3a7f3
  SOURCE_SUBDIR none)
# the debian cereal uses system rapidjson 1.1.0, which does not compile with gcc 14
FetchContent_Declare(cereal
  GIT_REPOSITORY https://github.com/USCiLab/cereal.git
  GIT_TAG 22a1b369f39be918ca79206a83c4facd759f9105
  SOURCE_SUBDIR none)
FetchContent_MakeAvailable(sophus magic_enum basalt_headers opengv cereal)

find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d)
find_package(fmt REQUIRED)
find_package(nlohmann_json REQUIRED)

file(GLOB_RECURSE OPENGV_SOURCES ${opengv_SOURCE_DIR}/src/*.cpp)
add_library(basalt_opengv STATIC ${OPENGV_SOURCES})
get_target_property(EIGEN_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(basalt_opengv SYSTEM PUBLIC ${opengv_SOURCE_DIR}/include ${EIGEN_INCLUDE_DIR}/unsupported)
target_compile_options(basalt_opengv PRIVATE -include cassert -w)
target_link_libraries(basalt_opengv PUBLIC Eigen3::Eigen)
if(BASALT_MARCH)
  target_compile_options(basalt_opengv PUBLIC -march=${BASALT_MARCH})
endif()
set_target_properties(basalt_opengv PROPERTIES POSITION_INDEPENDENT_CODE ON)

set(BASALT_CORE_SOURCES
  ${BASALT_SOURCE_DIR}/src/linearization/landmark_block.cpp
  ${BASALT_SOURCE_DIR}/src/linearization/linearization_abs_qr.cpp
  ${BASALT_SOURCE_DIR}/src/linearization/linearization_abs_sc.cpp
  ${BASALT_SOURCE_DIR}/src/linearization/linearization_base.cpp
  ${BASALT_SOURCE_DIR}/src/linearization/linearization_rel_sc.cpp
  ${BASALT_SOURCE_DIR}/src/optical_flow/optical_flow.cpp
  ${BASALT_SOURCE_DIR}/src/utils/keypoints.cpp
  ${BASALT_SOURCE_DIR}/src/utils/system_utils.cpp
  ${BASALT_SOURCE_DIR}/src/utils/time_utils.cpp
  ${BASALT_SOURCE_DIR}/src/utils/vio_config.cpp
  ${BASALT_SOURCE_DIR}/src/vi_estimator/ba_base.cpp
  ${BASALT_SOURCE_DIR}/src/vi_estimator/landmark_database.cpp
  ${BASALT_SOURCE_DIR}/src/vi_estimator/marg_helper.cpp
  ${BASALT_SOURCE_DIR}/src/vi_estimator/sc_ba_base.cpp
  ${BASALT_SOURCE_DIR}/src/vi_estimator/sqrt_ba_base.cpp
  ${BASALT_SOURCE_DIR}/src/vi_estimator/sqrt_keypoint_vio.cpp
  ${BASALT_SOURCE_DIR}/src/vi_estimator/sqrt_keypoint_vo.cpp
  ${BASALT_SOURCE_DIR}/src/vi_estimator/vio_estimator.cpp)

add_library(basalt_core STATIC ${BASALT_CORE_SOURCES})
target_include_directories(basalt_core SYSTEM PUBLIC
  ${BASALT_SOURCE_DIR}/include
  ${basalt_headers_SOURCE_DIR}/include
  ${sophus_SOURCE_DIR}
  ${magic_enum_SOURCE_DIR}/include
  ${cereal_SOURCE_DIR}/include)
target_compile_definitions(basalt_core PUBLIC
  BASALT_INSTANTIATIONS_DOUBLE
  BASALT_INSTANTIATIONS_FLOAT
  SOPHUS_USE_BASIC_LOGGING)
target_compile_options(basalt_core PRIVATE -w)
if(BASALT_MARCH)
  # eigen alignment must match between basalt and the node, so the flag is public
  target_compile_options(basalt_core PUBLIC -march=${BASALT_MARCH})
endif()
target_link_libraries(basalt_core
  PUBLIC Eigen3::Eigen TBB::tbb opencv_core opencv_imgproc opencv_features2d fmt::fmt
  PRIVATE basalt_opengv nlohmann_json::nlohmann_json)
set_target_properties(basalt_core PROPERTIES POSITION_INDEPENDENT_CODE ON CXX_STANDARD 17)
