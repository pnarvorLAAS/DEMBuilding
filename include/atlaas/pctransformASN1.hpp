#ifndef __PCTRANSFORMASN1_HPP__
#define __PCTRANSFORMASN1_HPP__

#include <memory>
#include "pctransform.hpp"
#include <PointCloud_InFuse.h>
#include <Pose_InFuse.h>
#include <conversions/asn1_conversions.hpp>
#include <Time.h>

#define DEFAULT_FIXED_FRAME "LocalTerrainFrame"

namespace atlaas{

class cloudTransformASN1: public cloudTransform
{
    private:
            
        std::shared_ptr<PointCloud_InFuse>      pcMsgInput; // Message to decode
        std::shared_ptr<PointCloud_InFuse>      pcMsgOutput; // Encoded message to publish
        std::unique_ptr<Pose_InFuse>            transformToWorld; // Transform from robot body frame to world
        byte*                   perBuffer; // Will be allocated once to the right message size (point cloud)
        byte*                   perBufferPose; // Will be allocated once to the right message size (pose)
        Time                    lastMsgTimeStamp;
            
    public:
        cloudTransformASN1();
        cloudTransformASN1(std::string worldFrame);
        void init();

        ~cloudTransformASN1();
        virtual void clean_up();

        bool decode_message(BitStream &msg);
        void print_inputMsg();
        void print_outputMsg();
        bool decode_pose(BitStream &msg);
        bool update_outputMsg(/*pcMsgOutput,pointCloud, tfSensor2World*/);
        bool update_transform(/*pcMsgInput,tfSensor2World*/);
        bool update_pointCloud(/*pcMsgInput,pointCloud*/);
        BitStream encode_message(/*pcMsgOutput*/);
        BitStream create_request(/*pcMsgInput*/);

        //Getter to avoid copy, should be used with caution
        const std::shared_ptr<PointCloud_InFuse> getOutput(){return pcMsgOutput;}

};

};

#endif
