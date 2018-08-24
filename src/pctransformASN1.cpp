#include <infuse_dem_building/pctransformASN1.hpp>

namespace atlaas{
    
    cloudTransformASN1::cloudTransformASN1()
    {
        init();
        fixedFrame = DEFAULT_FIXED_FRAME;
    }

    cloudTransformASN1::cloudTransformASN1(std::string worldFrame)
    {
        init();
        fixedFrame = worldFrame;
    }

    void cloudTransformASN1::init()
    {
        pcMsgInput = std::make_shared<PointCloud_InFuse>();
        pcMsgOutput = std::make_shared<PointCloud_InFuse>();
        transformToWorld = std::unique_ptr<Pose_InFuse>(new Pose_InFuse);

        lastMsgTimeStamp.microseconds = 0;
        lastMsgTimeStamp.usecPerSec = 0;

        perBuffer = (byte*) malloc(PointCloud_InFuse_REQUIRED_BYTES_FOR_ENCODING*sizeof(byte));
        memset(perBuffer,0,PointCloud_InFuse_REQUIRED_BYTES_FOR_ENCODING);

        perBufferPose = (byte*) malloc(Pose_InFuse_REQUIRED_BYTES_FOR_ENCODING*sizeof(byte));
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
    }
    
    bool cloudTransformASN1::decode_message(BitStream &msg)
    {
        int errorCode;

        if (!PointCloud_InFuse_Decode(pcMsgInput.get(),&msg,&errorCode))
        {
            std::cerr << "[Decoding] failed, error code: " << errorCode <<  std::endl;
            return false;
        }

        bool isTheSame = (lastMsgTimeStamp.microseconds == pcMsgInput->timeStamp.microseconds && lastMsgTimeStamp.usecPerSec == pcMsgInput->timeStamp.usecPerSec);

        lastMsgTimeStamp.microseconds = pcMsgInput->timeStamp.microseconds;
        lastMsgTimeStamp.usecPerSec = pcMsgInput->timeStamp.usecPerSec;

        return !isTheSame;
    }

    bool cloudTransformASN1::decode_pose(BitStream &msg)
    {
        int errorCode;
        if (!Pose_InFuse_Decode(transformToWorld.get(),&msg,&errorCode))
        {
            std::cerr << "[Decoding pose] failed, error code: " << errorCode << std::endl;
            return false;
        }

        

        // DEBUG ////////////////////////////////////

        //std::cout << "Input R2W Pose" << std::endl;
        //printPose(*transformToWorld.get());

        //std::cout << " Time stamps: " << std::endl;
        //std::cout << "[PARENT] " << transformToWorld->parentTime.microseconds << ", " << transformToWorld->parentTime.usecPerSec << std::endl;
        //std::cout << "[CHILD] " << transformToWorld->childTime.microseconds << ", " << transformToWorld->childTime.usecPerSec << std::endl;
        //std::cout << "Frames: " << std::endl;
        //std::cout << "[PARENT] " << transformToWorld->parentFrameId.arr << std::endl;
        //std::cout << "[CHILD] " << transformToWorld->childFrameId.arr << std::endl;

        ////////////////////////////////////////////
        return true;
    }

    
    bool cloudTransformASN1::update_transform(/*pointCloudMsg,tfSensor2World*/)
    {
        //Update Rotation from sensor to robot from msg
        rotationSensor2Robot = Eigen::Quaterniond(pcMsgInput->pose_robotFrame_sensorFrame.transform.orientation.arr[3],pcMsgInput->pose_robotFrame_sensorFrame.transform.orientation.arr[0],pcMsgInput->pose_robotFrame_sensorFrame.transform.orientation.arr[1],pcMsgInput->pose_robotFrame_sensorFrame.transform.orientation.arr[2]);

        //Convert to rotation matrix
        transformSensor2Robot.block<3,3>(0,0) = rotationSensor2Robot.normalized().toRotationMatrix();

        //Convert to homogeneous transformation
        transformSensor2Robot(0,3) = pcMsgInput->pose_robotFrame_sensorFrame.transform.translation.arr[0];
        transformSensor2Robot(1,3) = pcMsgInput->pose_robotFrame_sensorFrame.transform.translation.arr[1];
        transformSensor2Robot(2,3) = pcMsgInput->pose_robotFrame_sensorFrame.transform.translation.arr[2];
        transformSensor2Robot(3,3) = 1;

        //Update Rotation from robot to world from msg
        rotationRobot2World = Eigen::Quaterniond(transformToWorld->transform.orientation.arr[3],transformToWorld->transform.orientation.arr[0],transformToWorld->transform.orientation.arr[1],transformToWorld->transform.orientation.arr[2]);
        
        //Convert to rotation matrix
        transformRobot2World.block<3,3>(0,0) = rotationRobot2World.normalized().toRotationMatrix();

        for (int i = 0; i < 3; i++)
        {
            transformRobot2World(3,i) = 0;
            transformSensor2Robot(3,i) = 0;
        }

        //Convert to homogeneous transformation
        transformRobot2World(0,3) = transformToWorld->transform.translation.arr[0];
        transformRobot2World(1,3) = transformToWorld->transform.translation.arr[1];
        transformRobot2World(2,3) = transformToWorld->transform.translation.arr[2];
        transformRobot2World(3,3) = 1;

//        std::cout << "Cloud Transform R2W translation: " << transformRobot2World(0,3) << ", " << transformRobot2World(1,3) << ", " << transformRobot2World(2,3) << std::endl;


        //Compute total transform

        transformSensor2World =  transformRobot2World * transformSensor2Robot ;

	Eigen::Quaterniond ori(transformSensor2World.block<3,3>(0,0));

	//std::cout << "TRANSFORM S2W FROM PREVIOUS DATA" << std::endl;
	//std::cout << "== Position" << std::endl;
	//std::cout << "==== [";
	//std::cout << transformSensor2World(0,3) << ", " << transformSensor2World(1,3) << ", " << transformSensor2World(2,3);
	//std::cout << "]" << std::endl;
	//std::cout << "== Orientation " << std::endl;
	//std::cout << "==== [";
	//std::cout << "w = " << ori.w();
	//std::cout << ", x = " << ori.x();
	//std::cout << ", y = " << ori.y();
	//std::cout << ", z = " << ori.z();
	//std::cout << "]" << std::endl;

        //std::cout << "Transform:[  " << std::endl;
        for (int i=0; i<4;i++)
        {
            for (int j=0; j<4;j++)
            {
                tfSensor2World[i*4 + j] = transformSensor2World(i,j);
//                std::cout << tfSensor2World[i*4 + j] << ", " ;
            }
//            std::cout << std::endl;
        }
//        std::cout << "]" << std::endl;


//        std::cout << "Cloud Transform S2W translation: " << transformSensor2World(0,3) <<", " << transformSensor2World(1,3) << "," << transformSensor2World(2,3) << std::endl;
//        Eigen::Quaterniond ro(transformSensor2World.block<3,3>(0,0));
//        std::cout << "Cloud Transform S2W orientation: " << ro.w() <<", " << ro.x() << "," << ro.y() << ", " << ro.z() << std::endl;

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
            (*it)[3] = pcMsgInput->intensity.arr[i]; 
            it++;
        }
        return true;
    }

    bool cloudTransformASN1::update_outputMsg(/*pcMsgOutput,pointCloud,tfSensor2World*/)
    {
        // Change frameId
        std::string frameMsg = fixedFrame;
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
        //pcMsgOutput->pose_fixedFrame_robotFrame = pcMsgInput->pose_fixedFrame_robotFrame;


        pcMsgOutput->colors = pcMsgInput->colors;
        pcMsgOutput->intensity = pcMsgInput->intensity;

        //Update the pose
        pcMsgOutput->pose_fixedFrame_robotFrame = *transformToWorld.get();

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
        pcMsgOutput->intensity.nCount = pcMsgInput->points.nCount;
        pcMsgOutput->points.nCount = pcMsgInput->points.nCount;

        //std::cout << "========================= " << std::endl;
        //std::cout << "pcMsgOutput: pose R2W" << std::endl;
        //printPose(pcMsgOutput->pose_fixedFrame_robotFrame);

        return true;
    }
    
    BitStream cloudTransformASN1::encode_message(/*pcMsgOutput*/)
    {
        int errorCode;
        BitStream b;

        BitStream_Init(&b,perBuffer,PointCloud_InFuse_REQUIRED_BYTES_FOR_ENCODING);
        if (!PointCloud_InFuse_Encode(pcMsgOutput.get(),&b,&errorCode,TRUE))
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

    BitStream cloudTransformASN1::create_request(/*pcMsgInput*/)
    {
        int errorCode;
        BitStream b;

        /* Build the request */ 

        Pose_InFuse request;
        toASN1SCC(fixedFrame,request.parentFrameId);
        //request.childFrameId = pcMsgInput->pose_robotFrame_sensorFrame.parentFrameId;

        //TODO FIX THIS
        std::string childFrame("RoverBodyFrame");

        toASN1SCC(childFrame,request.childFrameId);
        request.msgVersion = pose_InFuse_Version;
        request.parentTime = pcMsgInput->timeStamp;
        request.childTime = pcMsgInput->timeStamp;
        request.transform = pcMsgInput->pose_robotFrame_sensorFrame.transform;

        /* Encoding */

        BitStream_Init(&b,perBufferPose,Pose_InFuse_REQUIRED_BYTES_FOR_ENCODING);

        if (!Pose_InFuse_Encode(&request,&b,&errorCode,TRUE))
        {
            std::cerr << "[Encoding of request] failed, error code: " << errorCode << std::endl;
            clean_up();
            exit(-1);
        }
        else
        {
            return b;
        }
    }

    void cloudTransformASN1::print_inputMsg()
    {
        std::cout << "pcMsgInput Metadata: " << std::endl;
        std::cout << "==== General Metadata: " << std::endl;
        std::cout << "======== Header frameId: " << pcMsgInput->frameId.arr << std::endl;
        std::cout << "======== nCount frameId: " << pcMsgInput->frameId.nCount << std::endl;
        std::cout << "======== timeStamp: [" << pcMsgInput->timeStamp.microseconds << ", " << pcMsgInput->timeStamp.usecPerSec << "]" <<  std::endl;

        std::cout << "==== On the pointCloud: " << std::endl;
        std::cout << "======== nCount pointCloud: " << pcMsgInput->points.nCount << std::endl;
        std::cout << "======== nCount colors: " << pcMsgInput->colors.nCount << std::endl;
        std::cout << "======== nCount intensity: " << pcMsgInput->intensity.nCount << std::endl;


        std::cout << "==== On the poses [Frame ids]: " << std::endl;
        std::cout << "======== World->robot [PARENT]: " <<  pcMsgInput->pose_fixedFrame_robotFrame.parentFrameId.arr << ",nCount: " << pcMsgInput->pose_fixedFrame_robotFrame.parentFrameId.nCount <<std::endl;
        std::cout << "======== World->robot[CHILD]: " <<  pcMsgInput->pose_fixedFrame_robotFrame.childFrameId.arr << ",nCount: " << pcMsgInput->pose_fixedFrame_robotFrame.childFrameId.nCount << std::endl;
        std::cout << "======== Robot->sensor[PARENT]: " << pcMsgInput->pose_robotFrame_sensorFrame.parentFrameId.arr << ", nCount: " << pcMsgInput->pose_robotFrame_sensorFrame.parentFrameId.nCount << std::endl;
        std::cout << "======== Robot->sensor[CHILD]: " << pcMsgInput->pose_robotFrame_sensorFrame.childFrameId.arr << ", nCount: " << pcMsgInput->pose_robotFrame_sensorFrame.childFrameId.nCount << std::endl;


        std::cout << "==== On the poses[Times]: " << std::endl;
        std::cout << "======== World -> Robot[PARENT]: " << pcMsgInput->pose_fixedFrame_robotFrame.parentTime.microseconds << ", " << pcMsgInput->pose_fixedFrame_robotFrame.parentTime.usecPerSec << std::endl;
        std::cout << "======== World -> Robot[CHILD]: " << pcMsgInput->pose_fixedFrame_robotFrame.childTime.microseconds << ", " << pcMsgInput->pose_fixedFrame_robotFrame.childTime.usecPerSec << std::endl;
        std::cout << "======== Robot -> sensor [PARENT]: " << pcMsgInput->pose_robotFrame_sensorFrame.parentTime.microseconds << ", " << pcMsgInput->pose_robotFrame_sensorFrame.parentTime.usecPerSec << std::endl;
        std::cout << "======== Robot -> sensor [CHILD]: " << pcMsgInput->pose_robotFrame_sensorFrame.childTime.microseconds << ", " << pcMsgInput->pose_robotFrame_sensorFrame.childTime.usecPerSec << std::endl;
    }
    
    void cloudTransformASN1::print_outputMsg()
    {
        std::cout << "pcMsgOutput Metadata: " << std::endl;
        std::cout << "==== General Metadata: " << std::endl;
        std::cout << "======== Header frameId: " << pcMsgOutput->frameId.arr << std::endl;
        std::cout << "======== nCount frameId: " << pcMsgOutput->frameId.nCount << std::endl;
        std::cout << "======== timeStamp: [" << pcMsgOutput->timeStamp.microseconds << ", " << pcMsgOutput->timeStamp.usecPerSec << "]" <<  std::endl;

        std::cout << "==== On the pointCloud: " << std::endl;
        std::cout << "======== nCount pointCloud: " << pcMsgOutput->points.nCount << std::endl;
        std::cout << "======== nCount colors: " << pcMsgOutput->colors.nCount << std::endl;
        std::cout << "======== nCount intensity: " << pcMsgOutput->intensity.nCount << std::endl;


        std::cout << "==== On the poses [Frame ids]: " << std::endl;
        std::cout << "======== World->robot [PARENT]: " <<  pcMsgOutput->pose_fixedFrame_robotFrame.parentFrameId.arr << ",nCount: " << pcMsgOutput->pose_fixedFrame_robotFrame.parentFrameId.nCount <<std::endl;
        std::cout << "======== World->robot[CHILD]: " <<  pcMsgOutput->pose_fixedFrame_robotFrame.childFrameId.arr << ",nCount: " << pcMsgOutput->pose_fixedFrame_robotFrame.childFrameId.nCount << std::endl;
        std::cout << "======== Robot->sensor[PARENT]: " << pcMsgOutput->pose_robotFrame_sensorFrame.parentFrameId.arr << ", nCount: " << pcMsgOutput->pose_robotFrame_sensorFrame.parentFrameId.nCount << std::endl;
        std::cout << "======== Robot->sensor[CHILD]: " << pcMsgOutput->pose_robotFrame_sensorFrame.childFrameId.arr << ", nCount: " << pcMsgOutput->pose_robotFrame_sensorFrame.childFrameId.nCount << std::endl;


        std::cout << "==== On the poses[Times]: " << std::endl;
        std::cout << "======== World -> Robot[PARENT]: " << pcMsgOutput->pose_fixedFrame_robotFrame.parentTime.microseconds << ", " << pcMsgOutput->pose_fixedFrame_robotFrame.parentTime.usecPerSec << std::endl;
        std::cout << "======== World -> Robot[CHILD]: " << pcMsgOutput->pose_fixedFrame_robotFrame.childTime.microseconds << ", " << pcMsgOutput->pose_fixedFrame_robotFrame.childTime.usecPerSec << std::endl;
        std::cout << "======== Robot -> sensor [PARENT]: " << pcMsgOutput->pose_robotFrame_sensorFrame.parentTime.microseconds << ", " << pcMsgOutput->pose_robotFrame_sensorFrame.parentTime.usecPerSec << std::endl;
        std::cout << "======== Robot -> sensor [CHILD]: " << pcMsgOutput->pose_robotFrame_sensorFrame.childTime.microseconds << ", " << pcMsgOutput->pose_robotFrame_sensorFrame.childTime.usecPerSec << std::endl;
    }
    
    void cloudTransformASN1::printPose(Pose_InFuse pose)
    {
        std::cout << "== Position" << std::endl;
        std::cout << "==== [";
        std::cout << pose.transform.translation.arr[0] << ", " << pose.transform.translation.arr[1] << ", " << pose.transform.translation.arr[2];
        std::cout << "]" << std::endl;
        std::cout << "== Orientation " << std::endl;
        std::cout << "==== [";
        std::cout << "w = " << pose.transform.orientation.arr[3];
        std::cout << ", x = " << pose.transform.orientation.arr[0];
        std::cout << ", y = " << pose.transform.orientation.arr[1];
        std::cout << ", z = " << pose.transform.orientation.arr[2];
        std::cout << "]" << std::endl;
    }
};
