#ifndef _PCTRANSFORM_HPP_
#define _PCTRANSFORM_HPP_

#include <PointCloudPoseStamped.h>
#include <atlaas/common.hpp>
#include <Eigen/Geometry>

namespace atlaas {

class cloudTransform
{
    private:

        /* Inputs and outputs of the DFN */
        points pointCloud; 
        matrix tfSensor2World;
        PointCloudPoseStamped pcMsgInput; // Message to decode
        PointCloudPoseStamped pcMsgOutput; // Encoded message to publish

        /* Internal variables */
        
        Eigen::Quaterniond q; // Store incoming quaternion into msg
        Eigen::Matrix3d rotation; // Store rotation matrix comupted from quaternion
        Eigen::Matrix4d homoTrans; // Store homogenous transformation



    public:
        bool decode_message(BitStream msg);
        bool update_transform(/*pointCloudMsg,tfSensor2World*/);
        bool update_pointCloud(/*pointCloudMsg,pointCloud*/);
        bool transform_pointCloud(/*pointCloud*/);
};

};


#endif
