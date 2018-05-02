#ifndef __PCTRANSFORMASN1_HPP__
#define __PCTRANSFORMASN1_HPP__

#include "pctransform.hpp"
#include <PointCloud_InFuse.h>
#include <conversions/asn1_conversions.hpp>
#include <Time.h>

#define DEFAULT_FIXED_FRAME "localTerrainFrame"

namespace atlaas{

class cloudTransformASN1: public cloudTransform
{
    private:
            
        PointCloud_InFuse*      pcMsgInput; // Message to decode
        PointCloud_InFuse*      pcMsgOutput; // Encoded message to publish
        byte*                   perBuffer; // Will be allocated once to the right message size
        Time                    lastMsgTimeStamp;
            
    public:
        cloudTransformASN1();
        ~cloudTransformASN1();
        virtual void clean_up();

        bool decode_message(BitStream msg);
        bool update_outputMsg(/*pcMsgOutput,pointCloud, tfSensor2World*/);
        bool update_transform(/*pcMsgInput,tfSensor2World*/);
        bool update_pointCloud(/*pcMsgInput,pointCloud*/);
        BitStream encode_message(/*pcMsgOutput*/);
};

};

#endif
