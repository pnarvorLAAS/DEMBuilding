#include <atlaas/pctransformASN1.hpp>

namespace atlaas{
    
    cloudTransformASN1::cloudTransformASN1()
    {
        pcMsgInput = new PointCloud_InFuse;
        pcMsgOutput = new PointCloud_InFuse;
        transformToWorld = new Pose_InFuse;

        lastMsgTimeStamp.microseconds = 0;
        lastMsgTimeStamp.usecPerSec = 0;

        perBuffer = (byte*) malloc(PointCloud_InFuse_REQUIRED_BYTES_FOR_ENCODING*sizeof(byte));
        perBufferPose = (byte*) malloc(Pose_InFuse_REQUIRED_BYTES_FOR_ENCODING*sizeof(byte));
        memset(perBuffer,0,PointCloud_InFuse_REQUIRED_BYTES_FOR_ENCODING);
        memset(perBufferPose,0,Pose_InFuse_REQUIRED_BYTES_FOR_ENCODING);
    }

    cloudTransformASN1::~cloudTransformASN1()
    {
        clean_up();
    }

    void cloudTransformASN1::clean_up()
    {
        std::cout << "Cleaning up the cloud ASN1 point cloud tranformer!" << std::endl;
        free(perBuffer);
        free(perBufferPose);
        delete pcMsgInput;
        delete pcMsgOutput;
        delete transformToWorld;
    }
    
    bool cloudTransformASN1::decode_message(BitStream msg)
    {
        int errorCode;

        if (!PointCloud_InFuse_Decode(pcMsgInput,&msg,&errorCode))
        {
            std::cerr << "[Decoding] failed, error code: " << errorCode <<  std::endl;
            return false;
        }

        bool isTheSame = (lastMsgTimeStamp.microseconds == pcMsgInput->timeStamp.microseconds && lastMsgTimeStamp.usecPerSec == pcMsgInput->timeStamp.usecPerSec);

        lastMsgTimeStamp.microseconds = pcMsgInput->timeStamp.microseconds;
        lastMsgTimeStamp.usecPerSec = pcMsgInput->timeStamp.usecPerSec;

        return !isTheSame;
    }

    bool cloudTransformASN1::decode_pose(BitStream msg)
    {
        int errorCode;
        if (!Pose_InFuse_Decode(transformToWorld,&msg,&errorCode))
        {
            std::cerr << "[Decoding pose] failed, error code: " << errorCode << std::endl;
            return false;
        }
        return true;
    }

    
    bool cloudTransformASN1::update_transform(/*pointCloudMsg,tfSensor2World*/)
    {
        //Update Rotation from sensor to robot from msg
        rotationSensor2Robot = Eigen::Quaterniond(pcMsgInput->pose_robotFrame_sensorFrame.transform.orientation.arr);

        //Convert to rotation matrix
        transformSensor2Robot.block<3,3>(0,0) = rotationSensor2Robot.normalized().toRotationMatrix();

        //Convert to homogeneous transformation
        transformSensor2Robot(0,3) = pcMsgInput->pose_robotFrame_sensorFrame.transform.translation.arr[0];
        transformSensor2Robot(1,3) = pcMsgInput->pose_robotFrame_sensorFrame.transform.translation.arr[1];
        transformSensor2Robot(2,3) = pcMsgInput->pose_robotFrame_sensorFrame.transform.translation.arr[2];

        //Update Rotation from robot to world from msg
        rotationRobot2World = Eigen::Quaterniond(transformToWorld->transform.orientation.arr);
        
        //Convert to rotation matrix
        transformRobot2World.block<3,3>(0,0) = rotationRobot2World.normalized().toRotationMatrix();

        //Convert to homogeneous transformation
        transformRobot2World(0,3) = transformToWorld->transform.translation.arr[0];
        transformRobot2World(1,3) = transformToWorld->transform.translation.arr[1];
        transformRobot2World(2,3) = transformToWorld->transform.translation.arr[2];

        //Compute total transform

        transformSensor2World = transformRobot2World * transformSensor2Robot;

        //std::cout << "Transform:[  " << std::endl;
        for (int i=0; i<4;i++)
        {
            for (int j=0; j<4;j++)
            {
                tfSensor2World[i*4 + j] = transformSensor2World(i,j);
                //std::cout << tfSensor2World[i*4 + j] << ", " ;
            }
            //std::cout << std::endl;
        }
        //std::cout << "]" << std::endl;


        return true;
    }
    
    bool cloudTransformASN1::update_pointCloud(/*pointCloudMsg,pointCloud*/)
    {
        pointCloud.resize(pcMsgInput->points.nCount);
        auto it = pointCloud.begin();
        for (int i=0; i < pcMsgInput->points.nCount; i++)
        {
            (*it)[0] = pcMsgInput->points.arr[i].arr[0]; 
            (*it)[1] = pcMsgInput->points.arr[i].arr[1]; 
            (*it)[2] = pcMsgInput->points.arr[i].arr[2]; 
            (*it)[3] = pcMsgInput->points.arr[i].arr[3]; 
            it++;
        }
        return true;
    }
    
    bool cloudTransformASN1::update_outputMsg(/*pcMsgOutput,pointCloud,tfSensor2World*/)
    {
        // Change frameId
        std::string frameMsg = DEFAULT_FIXED_FRAME;
        toASN1SCC(frameMsg,pcMsgOutput->frameId);

        /* Copy all other informations */
        pcMsgOutput->msgVersion = pcMsgInput->msgVersion;
        pcMsgOutput->sensorId = pcMsgInput->sensorId;
        pcMsgOutput->timeStamp = pcMsgInput->timeStamp;
        
        pcMsgOutput->isRegistered = pcMsgInput->isRegistered;
        pcMsgOutput->isOrdered = pcMsgInput->isOrdered;
        pcMsgOutput->height = pcMsgInput->height;
        pcMsgOutput->width = pcMsgInput->width;

        pcMsgOutput->hasFixedTransform = pcMsgInput->hasFixedTransform;
        pcMsgOutput->pose_robotFrame_sensorFrame = pcMsgInput->pose_robotFrame_sensorFrame;
        pcMsgOutput->pose_fixedFrame_robotFrame = pcMsgInput->pose_fixedFrame_robotFrame;

        pcMsgOutput->colors = pcMsgInput->colors;
        pcMsgOutput->intensity = pcMsgInput->intensity;


        //Update the point cloud
        auto it =pointCloud.begin();
        for (int i=0; i< pcMsgInput->points.nCount; i++)
        {
            pcMsgOutput->points.arr[i].arr[0] = (*it)[0];
            pcMsgOutput->points.arr[i].arr[1] = (*it)[1];
            pcMsgOutput->points.arr[i].arr[2] = (*it)[2];
            pcMsgOutput->points.arr[i].nCount = 3;
            it++;
        }
        pcMsgOutput->points.nCount = pcMsgInput->points.nCount;
        return true;

        /* DEBUG */

        //std::cout << "pcMsgOutput: " << std::endl;
        //std::cout << "nCount position: " << pcMsgInput->pose.pose.pos.nCount << std::endl;
        //std::cout << "nCount orientation: " << pcMsgInput->pose.pose.orient.nCount << std::endl;
        //std::cout << "nCount header string: " << pcMsgInput->header.frameId.nCount << std::endl;
        //std::cout << "header frameId: " << pcMsgInput->header.frameId.arr << std::endl;
        //std::cout << "nCount pointCloud: " << pcMsgInput->pointCloudData.nCount << std::endl;

        /* DEBUG */
    }
    
    BitStream cloudTransformASN1::encode_message(/*pcMsgOutput*/)
    {
        int errorCode;
        BitStream b;

        BitStream_Init(&b,perBuffer,PointCloud_InFuse_REQUIRED_BYTES_FOR_ENCODING);
        if (!PointCloud_InFuse_Encode(pcMsgOutput,&b,&errorCode,TRUE))
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


};
