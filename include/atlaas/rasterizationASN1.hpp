#ifndef __RASTERIZATIONASN1_HPP__
#define __RASTERIZATIONASN1_HPP__

#include <atlaas/rasterization.hpp>
#include <PointCloud_InFuse.h>
#include <DEM.h>

namespace atlaas{

    class pcRasterizerASN1: public pcRasterizer
    {
        private:
            PointCloud_InFuse*  pcMsgInput;
            DigitalElevationMap*    demMsgOutput;
            DigitalElevationRaster* demRasterMsgOutput;
            byte*                   perBuffer;
            byte*                   perBufferRaster;

        public:
            pcRasterizerASN1();
            ~pcRasterizerASN1();
            void clean_up();
            bool decode_message(BitStream msg);
            bool update_transform(/*pointCloudMsg,tfSensor2World*/);
            bool update_pointCloud(/*pointCloudMsg,pointCloud*/);
            bool update_outputMsg(/*demMsgOutput*/);
            bool update_rasterMsg(/*demRasterMsgOutput*/);
            BitStream encode_message(/*demMsgOutput, dyninter*/);
            BitStream encode_raster(/*demRasterMsgOutput*/);
    };
};

#endif
