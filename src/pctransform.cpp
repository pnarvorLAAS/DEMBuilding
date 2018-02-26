#ifndef _PCTRANSFORM_CPP
#define _PCTRANSFORM_CPP

#include <atlaas/pctransform.hpp>
#include <iostream>
#include <Eigen/Geometry>

namespace atlaas{

    bool cloudTransform::decode_message(BitStream msg)
    {
        BitStream b; /* Will serve to decode incoming bitstream msg */
        int errorCode;
        BitStream_AttachBuffer(&b,msg.buf,BitStream_GetLength(&msg));

        if (!PointCloudPoseStamped_Decode(&pointCloudMsg,&b,&errorCode))
        {
            std::cerr << "[Decoding] failed, error code: " << errorCode <<  std::endl;
            return false;
        }
        return true;
    }

    bool update_transform(/*pointCloudMsg,tfSensor2World*/)
    {
        Eigen::Matrix4d translation;
        return true;
    }
    bool update_pointCloud(/*pointCloudMsg,pointCloud*/)
    {
        return true;
    }
    bool transform_pointCloud(/*pointCloud*/)
    {
        return true;
    }
}

#endif
