#ifndef __RASTERIZATION_HPP__
#define __RASTERIZATION_HPP__

#include <gdalwrap/gdal.hpp>
#include <infuse_dem_building/common.hpp>
#include <Eigen/Geometry>

namespace atlaas{

    class pcRasterizer
    {
        protected:

            /* Inputs and outputs of the DFN */
            points                  pointCloud;
            matrix                  tfSensor2World;

            /* Internal variables */

            Eigen::Quaterniond      rotationSensor2Robot;   // Store rotation from sensor frame to rover body frame
            Eigen::Matrix4d         transformSensor2Robot;  // Store transform from sensor frame to rover body frame

            Eigen::Quaterniond      rotationRobot2World;    // Store rotation from rover body frame to fixed frame (Local terrain frame)
            Eigen::Matrix4d         transformRobot2World;   // Store transform from rover body frame to fixed frame (Local terrain frame)

            Eigen::Matrix4d         transformSensor2World;  // Store transform from sensor frame to fixed frame (Local Terrain frame)


            gdalwrap::gdal          meta;
            cells_info_t            dyninter;

            point_xy_t              sensor_xy;

            map_id_t                current;

            size_t                  width;
            size_t                  height;
            int                     sw;
            int                     sh;
            float                   scaleMap;

            uint64_t                time_base;



        public:
            pcRasterizer();
            ~pcRasterizer();
            virtual void clean_up();

            void init(double size_x,double size_y, double scale,
                    double custom_x, double custom_y, double custom_z,
                    int utm_zone, bool utm_north);

            bool rasterize(/*pointCloud*/);
            void do_slide();
            bool slide(/*sensor_xy, meta*/);
            void set_time_base(uint64_t base);
    };
}

#endif
