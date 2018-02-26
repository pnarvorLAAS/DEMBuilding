#ifndef _PCTRANSFORM_CPP
#define _PCTRANSFORM_CPP

#include <atlaas/pctransform.hpp>
#include <iostream>

namespace atlaas{

    bool cloudTransform::decode_message(BitStream msg)
    {
        BitStream b; /* Will serve to decode incoming bitstream msg */
        int errorCode;
        BitStream_AttachBuffer(&b,msg.buf,BitStream_GetLength(&msg));

        if (!PointCloudPoseStamped_Decode(&pcMsgInput,&b,&errorCode))
        {
            std::cerr << "[Decoding] failed, error code: " << errorCode <<  std::endl;
            return false;
        }
        return true;
    }

    bool cloudTransform::update_transform(/*pointCloudMsg,tfSensor2World*/)
    {
        //Update quaternion from msg
        q = Eigen::Quaterniond(pcMsgInput.pose.orient.arr);
        //Convert to rotation matrix
        homoTrans.block<3,3>(0,0) = q.normalized().toRotationMatrix();
        homoTrans(0,3) = pcMsgInput.pose.pos.arr[0];
        homoTrans(1,3) = pcMsgInput.pose.pos.arr[1];
        homoTrans(2,3) = pcMsgInput.pose.pos.arr[2];
        for (int i=0; i<4;i++)
        {
            for (int j=0; j<4;j++)
            {
                tfSensor2World[i*4 + j] = homoTrans(i,j);
            }
        }
        return true;
    }

    bool cloudTransform::update_pointCloud(/*pointCloudMsg,pointCloud*/)
    {
        pointCloud.resize(pcMsgInput.pointCloud.points.nCount);
        auto it = pointCloud.begin();
        for (int i=0; i < pcMsgInput.pointCloud.points.nCount; i++)
        {
           (*it)[0] = pcMsgInput.pointCloud.points.arr[i].arr[0]; 
           (*it)[1] = pcMsgInput.pointCloud.points.arr[i].arr[1]; 
           (*it)[2] = pcMsgInput.pointCloud.points.arr[i].arr[2]; 
           (*it)[3] = pcMsgInput.pointCloud.points.arr[i].arr[3]; 
        }
        return true;
    }
    bool cloudTransform::transform_pointCloud(/*pointCloud*/)
    {
        return true;
    }
}

#endif
