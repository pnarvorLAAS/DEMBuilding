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
        points                  pointCloud; 
        matrix                  tfSensor2World;
        PointCloudPoseStamped   pcMsgInput; // Message to decode
        PointCloudPoseStamped   pcMsgOutput; // Encoded message to publish

        /* Internal variables */
        
        Eigen::Quaterniond      q; // Store incoming quaternion into msg
        Eigen::Matrix3d         rotation; // Store rotation matrix computed from quaternion
        Eigen::Matrix4d         homoTrans; // Store homogenous transformation
        byte*                   perBuffer;

        /* Test variabes */

        int                     cloudSize;



    public:
        cloudTransform();
        ~cloudTransform();

        bool decode_message(BitStream msg);
        bool update_transform(/*pcMsgInput,tfSensor2World*/);
        bool update_pointCloud(/*pcMsgInput,pointCloud*/);
        bool transform_pointCloud(/*pointCloud*/);
        bool update_outputMsg(/*pcMsgOutput,pointCloud, tfSensor2World*/);
        BitStream encode_message(/*pcMsgOutput*/);

        /* Setters and getters for DEBUG */

        bool set_transform(matrix tfToSet)
        {
            tfSensor2World=tfToSet;
        };

        matrix get_transform()
        {
            return tfSensor2World;
        };

        bool set_pointCloud(points pcToSet,int size)
        {
            pointCloud=pcToSet;
            cloudSize = size;
        };

        points get_pointCloud()
        {
            return pointCloud;
        };

        int get_cloudSize()
        {
            return cloudSize;
        }
};

};


#endif
