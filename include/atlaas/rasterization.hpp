#ifndef __RASTERIZATION_HPP__
#define __RASTERIZATION_HPP__

#include <gdalwrap/gdal.hpp>
#include <atlaas/common.hpp>
#include <Eigen/Geometry>

namespace atlaas{

    class pcRasterizer
    {
        protected:

            /* Inputs and outputs of the DFN */
            points                  pointCloud;
            matrix                  tfSensor2World;

            /* Internal variables */

            Eigen::Quaterniond q; // Store incoming quaternion into msg
            Eigen::Matrix3d rotation; // Store rotation matrix computed from quaternion
            Eigen::Matrix4d homoTrans; // Store homogenous transformation

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
