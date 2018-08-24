#ifndef __PCTRANSFORMASN1_HPP__
#define __PCTRANSFORMASN1_HPP__

#include <memory>
#include <infuse_dem_building/pctransform.hpp>
#include <infuse_dem_building/PointCloud_InFuse.h>

#include <infuse_asn1_types/TransformWithCovariance.h>
#include <infuse_asn1_types/Time.h>

#include <infuse_asn1_conversions/asn1_conversions.hpp>

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

        //Helper function to print a Pose_InFuse 
        void printPose(Pose_InFuse pose); 

        //Getter to avoid copy, should be used with caution
        const std::shared_ptr<PointCloud_InFuse> getPcOutput(){return pcMsgOutput;}

};

};

#endif
