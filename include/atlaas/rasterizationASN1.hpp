#ifndef __RASTERIZATIONASN1_HPP__
#define __RASTERIZATIONASN1_HPP__

#include <atlaas/rasterization.hpp>
#include <memory>
#include <PointCloud_InFuse.h>
#include <DEM.h>

namespace atlaas{

    class pcRasterizerASN1: public pcRasterizer
    {
        private:
            std::shared_ptr<PointCloud_InFuse>  pcMsgInput;
            std::shared_ptr<DigitalElevationMap>    demMsgOutput;
            std::shared_ptr<DigitalElevationRaster> demRasterMsgOutput;

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

            void setPcInput(const std::shared_ptr<PointCloud_InFuse> ptr){pcMsgInput = ptr;}
            const std::shared_ptr<DigitalElevationMap> getLocalMap(){return demMsgOutput;}

            void print_inputMsg();
            void printPose(Pose_InFuse pose);
    };
};

#endif
