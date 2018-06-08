#ifndef __PCTRANSFORMASN1_HPP__
#define __PCTRANSFORMASN1_HPP__

#include "pctransform.hpp"
#include <PointCloud_InFuse.h>
#include <Pose_InFuse.h>
#include <conversions/asn1_conversions.hpp>
#include <Time.h>

#define DEFAULT_FIXED_FRAME "localTerrainFrame"

namespace atlaas{

class cloudTransformASN1: public cloudTransform
{
    private:
            
        PointCloud_InFuse*      pcMsgInput; // Message to decode
        PointCloud_InFuse*      pcMsgOutput; // Encoded message to publish
        Pose_InFuse*            transformToWorld; // Transform from robot body frame to world
        byte*                   perBuffer; // Will be allocated once to the right message size (point cloud)
        byte*                   perBufferPose; // Will be allocated once to the right message size (pose)
        Time                    lastMsgTimeStamp;
            
    public:
        cloudTransformASN1();
        cloudTransformASN1(std::string worldFrame);
        ~cloudTransformASN1();
        virtual void clean_up();

        bool decode_message(BitStream msg);
        bool decode_pose(BitStream msg);
        bool update_outputMsg(/*pcMsgOutput,pointCloud, tfSensor2World*/);
        bool update_transform(/*pcMsgInput,tfSensor2World*/);
        bool update_pointCloud(/*pcMsgInput,pointCloud*/);
        BitStream encode_message(/*pcMsgOutput*/);
        BitStream create_request(/*pcMsgInput*/);

        std::string getRobotFrame()
        {
            std::string frame;
            fromASN1SCC(pcMsgOutput->pose_robotFrame_sensorFrame.parentFrameId,frame);
            return frame;
        }

        Time getPointCloudTime()
        {
            return pcMsgInput->timeStamp;
        }
};

};

#endif
