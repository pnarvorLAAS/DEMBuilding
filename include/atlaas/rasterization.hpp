#ifndef __RASTERIZATION_HPP__
#define __RASTERIZATION_HPP__

#include <PointCloudPoseStamped.h>
#include <gdalwrap/gdal.hpp>
#include <atlaas/common.hpp>

namespace atlaas{

    class pcRasterizer
    {
        private:

            /* Inputs and outputs of the DFN */
            points                  pointCloud;
            matrix                  tfSensor2World;
            PointCloudPoseStamped   pcMsgInput;
            // demRaster            demMsgOutPut;

            /* Internal variables */

            gdalwrap::gdal          meta;
            cells_info_t            dyninter;

            point_xy_t              sensor_xy;

            map_id_t                current;

            size_t                  width;
            size_t                  height;
            int                     sw;
            int                     sh;

            uint64_t                time_base;



        public:

            bool decode_message(BitStream msg);
            void init(double size_x,double size_y, double scale,
                    double custom_x, double custom_y, double custom_z,
                    int utm_zone, bool utm_north);
            bool rasterize(/*pointCloud*/);
            void do_slide();
            bool slide(/*sensor_xy, meta*/);
            void set_time_base(uint64_t base);
            BitStream encode_message(/*demMsgOutput*/);


    };
}

#endif
