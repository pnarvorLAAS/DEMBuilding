#ifndef _PCTRANSFORM_CPP
#define _PCTRANSFORM_CPP

#include <atlaas/pctransform.hpp>
#include <iostream>

namespace atlaas{

    cloudTransform::cloudTransform()
    {
        std::cout << "cloudTransform created" << std::endl;
        perBuffer = (byte*) malloc(PointCloudPoseStamped_REQUIRED_BYTES_FOR_ENCODING*sizeof(byte));
        pcMsgInput = new PointCloudPoseStamped;
        pcMsgOutput = new PointCloudPoseStamped;

        memset(perBuffer,0,PointCloudPoseStamped_REQUIRED_BYTES_FOR_ENCODING);
        lastMsgTimeStamp.microseconds = 0;
        lastMsgTimeStamp.usecPerSec = 0;
    }

    cloudTransform::~cloudTransform()
    {
        clean_up();
    }

    void cloudTransform::clean_up()
    {
        std::cout << "Cleaning up the cloud tranformer!" << std::endl;
        free(perBuffer);
        delete pcMsgInput;
        delete pcMsgOutput;
    }



    bool cloudTransform::decode_message(BitStream msg)
    {
        BitStream b; /* Will serve to decode incoming bitstream msg */
        int errorCode;
        BitStream_AttachBuffer(&b,msg.buf,BitStream_GetLength(&msg));

        if (!PointCloudPoseStamped_Decode(pcMsgInput,&b,&errorCode))
        {
            std::cerr << "[Decoding] failed, error code: " << errorCode <<  std::endl;
            return false;
        }
        bool isTheSame = (lastMsgTimeStamp.microseconds == pcMsgInput->header.timeStamp.microseconds && lastMsgTimeStamp.usecPerSec == pcMsgInput->header.timeStamp.usecPerSec);
        return !isTheSame;
    }

    bool cloudTransform::update_transform(/*pointCloudMsg,tfSensor2World*/)
    {
        //Update quaternion from msg
        q = Eigen::Quaterniond(pcMsgInput->pose.pose.orient.arr);
        //Convert to rotation matrix
        homoTrans.block<3,3>(0,0) = q.normalized().toRotationMatrix();
        homoTrans(0,3) = pcMsgInput->pose.pose.pos.arr[0];
        homoTrans(1,3) = pcMsgInput->pose.pose.pos.arr[1];
        homoTrans(2,3) = pcMsgInput->pose.pose.pos.arr[2];

        std::cout << "Transform:[  " << std::endl;
        for (int i=0; i<4;i++)
        {
            for (int j=0; j<4;j++)
            {
                tfSensor2World[i*4 + j] = homoTrans(i,j);
                std::cout << tfSensor2World[i*4 + j] << ", " ;
            }
            std::cout << std::endl;
        }
        std::cout << "]" << std::endl;
        return true;
    }

    bool cloudTransform::update_pointCloud(/*pointCloudMsg,pointCloud*/)
    {
        pointCloud.resize(pcMsgInput->pointCloudData.nCount);
        auto it = pointCloud.begin();
        for (int i=0; i < pcMsgInput->pointCloudData.nCount; i++)
        {
            (*it)[0] = pcMsgInput->pointCloudData.arr[i].arr[0]; 
            (*it)[1] = pcMsgInput->pointCloudData.arr[i].arr[1]; 
            (*it)[2] = pcMsgInput->pointCloudData.arr[i].arr[2]; 
            (*it)[3] = pcMsgInput->pointCloudData.arr[i].arr[3]; 
            it++;
        }
        return true;
    }

    bool cloudTransform::transform_pointCloud(/*pointCloud*/)
    {
        float x,y,z;
        for (auto& point : pointCloud) 
        {
            x = point[0];
            y = point[1];
            z = point[2];
            point[0] = (x * tfSensor2World[0]) + (y * tfSensor2World[1]) + (z * tfSensor2World[2])  + tfSensor2World[3];
            point[1] = (x * tfSensor2World[4]) + (y * tfSensor2World[5]) + (z * tfSensor2World[6])  + tfSensor2World[7];
            point[2] = (x * tfSensor2World[8]) + (y * tfSensor2World[9]) + (z * tfSensor2World[10]) + tfSensor2World[11];
        }
        return true;
    }

    bool cloudTransform::update_outputMsg(/*pcMsgOutput,pointCloud,tfSensor2World*/)
    {
        pcMsgOutput->pose = pcMsgInput->pose;
        pcMsgOutput->header = pcMsgInput->header;
        auto it =pointCloud.begin();
        for (int i=0; i< pcMsgInput->pointCloudData.nCount; i++)
        {
            pcMsgOutput->pointCloudData.arr[i].arr[0] = (*it)[0];
            pcMsgOutput->pointCloudData.arr[i].arr[1] = (*it)[1];
            pcMsgOutput->pointCloudData.arr[i].arr[2] = (*it)[2];
            pcMsgOutput->pointCloudData.arr[i].nCount = 3;
            it++;
        }
        pcMsgOutput->pointCloudData.nCount = pcMsgInput->pointCloudData.nCount;

        /* DEBUG */

        //std::cout << "pcMsgOutput: " << std::endl;
        //std::cout << "nCount position: " << pcMsgInput->pose.pose.pos.nCount << std::endl;
        //std::cout << "nCount orientation: " << pcMsgInput->pose.pose.orient.nCount << std::endl;
        //std::cout << "nCount header string: " << pcMsgInput->header.frameId.nCount << std::endl;
        //std::cout << "header frameId: " << pcMsgInput->header.frameId.arr << std::endl;
        //std::cout << "nCount pointCloud: " << pcMsgInput->pointCloudData.nCount << std::endl;

        /* DEBUG */
    }
        
    BitStream cloudTransform::encode_message(/*pcMsgOutput*/)
    {
        int errorCode;
        BitStream b;

        BitStream_Init(&b,perBuffer,PointCloudPoseStamped_REQUIRED_BYTES_FOR_ENCODING);
        if (!PointCloudPoseStamped_Encode(pcMsgOutput,&b,&errorCode,TRUE))
        {
            std::cerr << "[Encoding] failed, error code: " << errorCode << std::endl;
            clean_up();
            exit(-1);
            
        }
        else
        {
            return b;
        }

    }


}

#endif
