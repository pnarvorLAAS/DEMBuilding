#ifndef __PCTRANSFORMASN1_HPP__
#define __PCTRANSFORMASN1_HPP__

#include "pctransform.hpp"
#include <PointCloudPoseStamped.h>
#include <Time.h>

namespace atlaas{

class cloudTransformASN1: public cloudTransform
{
    private:
            
        PointCloudPoseStamped*  pcMsgInput; // Message to decode
        PointCloudPoseStamped*  pcMsgOutput; // Encoded message to publish
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
