#include <iostream>
#include <infuse_dem_building/pcd.hpp>

namespace dem_building{
   
    void read_pcd_file(const std::string& filepath, points& cloud,
            matrix& transformation) {
        pcl::PointCloud<pcl::PointXYZI> input;
        pcl::io::loadPCDFile(filepath, input);
        cloud.resize( input.points.size() );
        auto it = cloud.begin();
        for (const auto& point : input.points) {
            (*it)[0] = point.x;
            (*it)[1] = point.y;
            (*it)[2] = point.z;
            (*it)[3] = point.intensity;
            ++it;
        }
        transformation = {{ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1, }}; // identity
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>
            eigen_matrix((double*)transformation.data());
        eigen_matrix.topLeftCorner<3,3>() =
            input.sensor_orientation_.toRotationMatrix().cast<double>();
        eigen_matrix.topRightCorner<4,1>() =
            input.sensor_origin_.cast<double>();
    }
    
    void write_pcd(const std::string& filepath, const points& cloud,
            const matrix& transformation) {
        pcl::PointCloud<pcl::PointXYZI> output;
        output.height = 1;
        output.width = cloud.size();
        output.is_dense = true;
        output.points.resize( cloud.size() );
        auto it = output.points.begin();
        for (const auto& point : cloud) {
            (*it).x = point[0];
            (*it).y = point[1];
            (*it).z = point[2];
            (*it).intensity = point[3];
            ++it;
        }
        // set transformation sensor-world
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>
            eigen_matrix((double*)transformation.data());
        output.sensor_orientation_ =
            Eigen::Quaternionf( eigen_matrix.topLeftCorner<3,3>().cast<float>() );
        output.sensor_origin_ =
            Eigen::Vector4f( eigen_matrix.topRightCorner<4,1>().cast<float>() );
        // save pcd
        pcl::io::savePCDFileBinaryCompressed(filepath, output);
    }
};
