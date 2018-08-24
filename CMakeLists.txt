cmake_minimum_required(VERSION 3.0)
project(infuse_dem_building VERSION "0.1")

set(CMAKE_CXX_STANDARD 11)
set(PKG_DESCRIPTION "Digital Elevation Model (DEM) building library")

# This is here to enabable cmake to find GSL and GdalWrap
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")

find_package(GdalWrap REQUIRED)
find_package(GSL REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

set(REQUIRED_COMPONENTS "eigen3, gsl, gdalwrap")

option(WITH_ASN1 "Compile the ASN.1 Wrapper of infuse_dem_building" TRUE)

if (WITH_ASN1)
    find_package(infuse_asn1_types REQUIRED)
    find_package(infuse_asn1_conversions REQUIRED)
    
    set(SOURCES
        src/pctransform.cpp
        src/rasterization.cpp
        src/fuse.cpp
        src/pctransformASN1.cpp
        src/rasterizationASN1.cpp
        #src/fuseASN1.cpp
        )

    set(HEADERS
        include/infuse_dem_building/pctransform.hpp
        include/infuse_dem_building/rasterization.hpp
        include/infuse_dem_building/fuse.hpp
        include/infuse_dem_building/pctransformASN1.hpp
        include/infuse_dem_building/rasterizationASN1.hpp
        #include/infuse_dem_building/fuseASN1.hpp
        )

    add_library(${PROJECT_NAME} SHARED ${SOURCES})
    target_link_libraries(${PROJECT_NAME}  ${GDALWRAP_LIBRARIES} ${GSL_LIBRARIES} infuse_asn1_conversions::base_conversions infuse_asn1_types::infuse_asn1_types)
    target_include_directories(${PROJECT_NAME} PUBLIC ${GDALWRAP_INCLUDE_DIRS} ${GSL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} PRIVATE include/)
    set_target_properties(${PROJECT_NAME}
                            PROPERTIES
                                VERSION ${PROJECT_VERSION}
                                OUTPUT_NAME ${PROJECT_NAME}
                                PUBLIC_HEADER "${HEADERS}"
                                )

    string(APPEND REQUIRED_COMPONENTS ", infuse_asn1_types, infuse_asn1_conversions_pom")

else()
    message(STATUS "\n\n[ATTENTION] You're currently compiling CRISP without ASN.1 support. This is not supported at the moment.\n\n")

endif(WITH_ASN1)

# Introduce variables:
# * CMAKE_INSTALL_LIBDIR
# * CMAKE_INSTALL_BINDIR
# * CMAKE_INSTALL_INCLUDEDIR
include(GNUInstallDirs)


# Layout. This works for all platforms:
#   * <prefix>/lib*/cmake/<PROJECT-NAME>
#   * <prefix>/lib*/
#   * <prefix>/include/
set(config_install_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

# This is exclusive for pkg-config
set(pkgconfig_install_dir "${CMAKE_INSTALL_LIBDIR}/pkgconfig")


set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")


# Configuration
set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
set(pkgconfig_file "${generated_dir}/${PROJECT_NAME}.pc")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
#
#
# Include module with fuctions:
#   * 'write_basic_package_version_file'
#   * 'configure_package_config_file'
include(CMakePackageConfigHelpers)
#
#
# Configure '<PROJECT-NAME>ConfigVersion.cmake'
# Uses:
#   * PROJECT_VERSION
write_basic_package_version_file(
  "${version_config}" COMPATIBILITY SameMajorVersion
)

# Configure '<PROJECT-NAME>Config.cmake'
# Uses variables:
#   * TARGETS_EXPORT_NAME
#   * PROJECT_NAME
#   * CMAKE_INSTALL_INCLUDEDIR
configure_package_config_file(
    "cmake/Config.cmake.in"
    "${project_config}"
    INSTALL_DESTINATION "${config_install_dir}"
    PATH_VARS
      CMAKE_INSTALL_INCLUDEDIR
)

# Configure '<PROJECT-NAME>.pc'
# Uses variables:
#   * PROJECT_NAME
#   * PROJECT_VERSION
#   * CMAKE_INSTALL_PREFIX
#   * CMAKE_INSTALL_LIBDIR
#   * CMAKE_INSTALL_INCLUDEDIR
configure_file(
    "cmake/pkgconfig.pc.cmakein"
    "${pkgconfig_file}" @ONLY
)


####
# Installation


# Targets:
#   * <prefix>/lib/lib${PROJECT_NAME}.so
#   * header location after install: <prefix>/include/${PROJECT_NAME}/*.h
#   * headers can be included by C++ code `#include <${PROJECT_NAME}/*.h>`
install(
    TARGETS ${PROJECT_NAME}
    EXPORT "${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}"
)


# Config
#   * <prefix>/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake
#   * <prefix>/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake
install(
    FILES "${project_config}" "${version_config}"
    DESTINATION "${config_install_dir}"
)


# Config
#   * <prefix>/lib/pkgconfig/${PROJECT_NAME}.pc
install(
    FILES "${pkgconfig_file}"
    DESTINATION "${pkgconfig_install_dir}")


# Config
#   * <prefix>/lib/cmake/infuse_cdff_types/infuse_cdff_typesTargets.cmake
install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    DESTINATION "${config_install_dir}"
)
