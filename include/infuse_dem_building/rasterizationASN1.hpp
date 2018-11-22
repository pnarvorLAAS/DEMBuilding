#ifndef __RASTERIZATIONASN1_HPP__
#define __RASTERIZATIONASN1_HPP__

#include <infuse_dem_building/rasterization.hpp>
#include <memory>
#include <infuse_asn1_types/Pointcloud.h>
#include <infuse_asn1_types/Map.h>
#include <infuse_asn1_conversions/asn1_base_conversions.hpp>
//#include <infuse_asn1_types/MultiLayeredMap.h>

namespace dem_building{

class pcRasterizerASN1: public pcRasterizer
{
    private:
            std::shared_ptr<asn1SccPointcloud>  pcMsgInput;
            std::shared_ptr<asn1SccMap>    demMsgOutput;
            std::shared_ptr<asn1SccMap> demRasterMsgOutput;

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

            void setPcInput(const std::shared_ptr<asn1SccPointcloud> ptr){pcMsgInput = ptr;}
            const std::shared_ptr<asn1SccMap> getLocalMap(){return demMsgOutput;}

            void print_inputMsg();
            void printPose(asn1SccTransformWithCovariance pose);
    };
};

#endif
