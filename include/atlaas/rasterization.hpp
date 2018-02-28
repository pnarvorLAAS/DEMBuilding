#ifndef __RASTERIZATION_HPP__
#define __RASTERIZATION_HPP__

#include <PointCloudPoseStamped.h>
#include <gdalwrap/gdal.h>
#include <common.hpp>

class pcRasterizer{
    private:

        /* Inputs and outputs of the DFN */
        points                  pointCloud;
        matrix                  tfSensor2World;
        PointCloudPoseStamped   pcMsgInput;
        // demRaster            demMsgOutPut;

        /* Internal variables */

        gdalwrap::gdal          meta;
        cells_info_t            dnyinter;

        point_xy_t              sensor_xy;

        size_t                  width;
        size_t                  height;
        int                     sw;
        int                     sh;



    public:

        bool decode_message(BitStream msg);
        void init(double size_x,double size_y, double scale,
                  double custom_x, double custom_y, double custom_z,
                  int utm_zone, boot utm_north=true);
        bool rasterize(/*pointCloud*/);
        void do_slide();
        bool slide(/*sensor_xy, meta*/);
        BitStream encode_message(/*demMsgOutput*/);


};

#endif
