cmake_minimum_required(VERSION 3.22.1)

project(ne_BallisticModel LANGUAGES CXX)

# ---- Options ----
option(BUILD_EXAMPLES "Build example targets" ON)

# ---- C++ standard ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- Dependencies ----
# Eigen is header-only; we link the imported target for include dirs and usage requirements.
find_package(Eigen3 REQUIRED)

# ---- Sources ----
file(GLOB_RECURSE NE_BM_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cxx")

set(NE_BM_PUBLIC_HEADERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

# ---- Library ----
if(NE_BM_SOURCES)
  add_library(ne_ballistic_model STATIC ${NE_BM_SOURCES})
  target_include_directories(ne_ballistic_model PUBLIC ${NE_BM_PUBLIC_HEADERS_DIR})
  target_compile_features(ne_ballistic_model PUBLIC cxx_std_17)
  target_link_libraries(ne_ballistic_model PUBLIC Eigen3::Eigen)
else()
  message(STATUS "No source files found in src/. Library target not created.")
endif()

# ---- Examples ----
if(BUILD_EXAMPLES)
  file(GLOB_RECURSE NE_BM_EXAMPLE_SOURCES CONFIGURE_DEPENDS
       "${CMAKE_CURRENT_SOURCE_DIR}/example/*.cpp"
       "${CMAKE_CURRENT_SOURCE_DIR}/example/*.cc"
       "${CMAKE_CURRENT_SOURCE_DIR}/example/*.cxx")

  if(NE_BM_EXAMPLE_SOURCES)
    add_executable(ne_ballistic_model_example ${NE_BM_EXAMPLE_SOURCES})
    target_include_directories(ne_ballistic_model_example PRIVATE ${NE_BM_PUBLIC_HEADERS_DIR})
    target_link_libraries(ne_ballistic_model_example PRIVATE Eigen3::Eigen)

    if(TARGET ne_ballistic_model)
      target_link_libraries(ne_ballistic_model_example PRIVATE ne_ballistic_model)
    endif()
  else()
    message(STATUS "No example sources found in example/. Example target not created.")
  endif()
endif()
