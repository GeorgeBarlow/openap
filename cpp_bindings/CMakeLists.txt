cmake_minimum_required(VERSION 3.14)
project(openap_trajectory_predictor VERSION 1.0.0 LANGUAGES CXX)

# =============================================================================
# C++ Standard
# =============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# =============================================================================
# Options
# =============================================================================
option(BUILD_EXAMPLES "Build example applications" ON)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# =============================================================================
# Find Python and pybind11
# =============================================================================
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG QUIET)

if(NOT pybind11_FOUND)
    message(STATUS "pybind11 not found via find_package, fetching from GitHub...")
    include(FetchContent)
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG v2.11.1
    )
    FetchContent_MakeAvailable(pybind11)
endif()

message(STATUS "Python executable: ${Python3_EXECUTABLE}")
message(STATUS "Python include dirs: ${Python3_INCLUDE_DIRS}")
message(STATUS "Python libraries: ${Python3_LIBRARIES}")

# =============================================================================
# Library: openap_trajectory
# =============================================================================
add_library(openap_trajectory
    src/trajectory_predictor.cpp
)

target_include_directories(openap_trajectory
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${Python3_INCLUDE_DIRS}
)

target_link_libraries(openap_trajectory
    PUBLIC
        pybind11::embed
        Python3::Python
)

# Copy Python binding module to build directory
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/trajectory_predictor_binding.py
    ${CMAKE_CURRENT_BINARY_DIR}/trajectory_predictor_binding.py
    COPYONLY
)

# Also copy the main prediction module
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../examples/trajectory_prediction_example.py)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/../examples/trajectory_prediction_example.py
        ${CMAKE_CURRENT_BINARY_DIR}/trajectory_prediction_example.py
        COPYONLY
    )
endif()

# =============================================================================
# Example executable
# =============================================================================
if(BUILD_EXAMPLES)
    add_executable(trajectory_example
        examples/trajectory_example.cpp
    )

    target_link_libraries(trajectory_example
        PRIVATE
            openap_trajectory
    )

    # Set RPATH so example can find Python libs
    set_target_properties(trajectory_example PROPERTIES
        BUILD_RPATH "${Python3_LIBRARY_DIRS}"
        INSTALL_RPATH "${Python3_LIBRARY_DIRS}"
    )
endif()

# =============================================================================
# Installation
# =============================================================================
include(GNUInstallDirs)

install(TARGETS openap_trajectory
    EXPORT openap_trajectory-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES
    src/trajectory_predictor_binding.py
    DESTINATION ${CMAKE_INSTALL_DATADIR}/openap
)

# =============================================================================
# Package configuration
# =============================================================================
install(EXPORT openap_trajectory-targets
    FILE openap_trajectory-targets.cmake
    NAMESPACE openap::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openap_trajectory
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/openap_trajectory-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/openap_trajectory-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openap_trajectory
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/openap_trajectory-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/openap_trajectory-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/openap_trajectory-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openap_trajectory
)
