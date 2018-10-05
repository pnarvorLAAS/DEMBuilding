#ifndef __DEM_BUILDING_PCD_HPP__
#define __DEM_BUILDING_PCD_HPP__

#include <infuse_dem_building/common.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace dem_building{

    void read_pcd_file(const std::string& filepath, points& cloud, matrix& transformation);
    void write_pcd(const std::string& filepath, const points& cloud, const matrix& transformation); 

};
#endif
