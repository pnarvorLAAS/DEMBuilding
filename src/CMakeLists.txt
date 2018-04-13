file(GLOB atlaas_SRCS "*.cpp")
add_library( atlaas SHARED ${atlaas_SRCS} )
target_link_libraries( atlaas ${GDALWRAP_LIBRARIES} )
target_link_libraries( atlaas ${GSL_LIBRARIES} )
target_link_libraries( atlaas ${PCL_LIBRARIES})
target_link_libraries( atlaas cdff_types)

set(DEPENDENCIES "gdal gdalwrap gsl eigen3 cdff_types")
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/atlaas.pc @ONLY) 


set(DEST_DIR "${CMAKE_INSTALL_PREFIX}")


#install(TARGETS atlaas DESTINATION lib)
#install_pkg_config_file(atlaas
#    DESCRIPTION "Atlas at LAAS"
#    CFLAGS
#    LIBS -latlaas
#    REQUIRES gdalwrap gsl
#    VERSION ${PACKAGE_VERSION})
