#include <infuse_dem_building/pctransformASN1.hpp>

namespace dem_building{
    
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
        pcMsgInput = std::make_shared<asn1SccPointcloud>();
        pcMsgOutput = std::make_shared<asn1SccPointcloud>();
        transformToWorld = std::unique_ptr<asn1SccTransformWithCovariance>(new asn1SccTransformWithCovariance);

        lastMsgTimeStamp.microseconds = 0;
        lastMsgTimeStamp.usecPerSec = 0;

        perBuffer = (byte*) malloc(asn1SccPointcloud_REQUIRED_BYTES_FOR_ENCODING*sizeof(byte));
        memset(perBuffer,0,asn1SccPointcloud_REQUIRED_BYTES_FOR_ENCODING);

        perBufferPose = (byte*) malloc(asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING*sizeof(byte));
        memset(perBufferPose,0,asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);
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

        if (!asn1SccPointcloud_Decode(pcMsgInput.get(),&msg,&errorCode))
        {
            std::cerr << "[Decoding] failed, error code: " << errorCode <<  std::endl;
            return false;
        }

        bool isTheSame = (lastMsgTimeStamp.microseconds == pcMsgInput->metadata.timeStamp.microseconds && lastMsgTimeStamp.usecPerSec == pcMsgInput->metadata.timeStamp.usecPerSec);

        lastMsgTimeStamp.microseconds = pcMsgInput->metadata.timeStamp.microseconds;
        lastMsgTimeStamp.usecPerSec = pcMsgInput->metadata.timeStamp.usecPerSec;

        return !isTheSame;
    }

    bool cloudTransformASN1::decode_pose(BitStream &msg)
    {
        int errorCode;
        if (!asn1SccTransformWithCovariance_Decode(transformToWorld.get(),&msg,&errorCode))
        {
            std::cerr << "[Decoding pose] failed, error code: " << errorCode << std::endl;
            return false;
        }

        // DEBUG ////////////////////////////////////
        
        
        std::cout << "[Pc Transform] Robot pose in world: [" << transformToWorld->data.translation.arr[0] << " " << transformToWorld->data.translation.arr[1]  << " " << transformToWorld->data.translation.arr[2] << "]" << std::endl;
        // check if world pose is equal to the one inside the velodyne message

        //if (transformToWorld->data.translation.arr[0] == pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.translation.arr[0] && transformToWorld->data.translation.arr[1] == pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.translation.arr[1] && transformToWorld->data.translation.arr[2] == pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.translation.arr[2])
        //{
        //    std::cout << "Positions are equal" << std::endl;
        //}
        //else
        //{
        //    std::cout << "Positions are different : " << std::endl;
        //    std::cout << "Message: [" <<  pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.translation.arr[0] << " " << pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.translation.arr[1] << " " << pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.translation.arr[2] << "]" << std::endl;
        //    std::cout << "Request: [" << transformToWorld->data.translation.arr[0] << " " << transformToWorld->data.translation.arr[1]  << " " << transformToWorld->data.translation.arr[2] << "]" << std::endl;
        //}


        //if (transformToWorld->data.orientation.arr[0] == pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.orientation.arr[0] && transformToWorld->data.orientation.arr[1] == pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.orientation.arr[1] && transformToWorld->data.orientation.arr[2] == pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.orientation.arr[2] && transformToWorld->data.orientation.arr[3] == pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.orientation.arr[3])
        //{
        //    std::cout << "Orientations are equal" << std::endl;
        //}
        //else
        //{
        //    std::cout << "Orientations are different: " << std::endl;
        //    std::cout << "Message: [" <<  pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.orientation.arr[0] << " " << pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.orientation.arr[1] << " " << pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.orientation.arr[2] << " " << pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.orientation.arr[3] << "]" << std::endl;
        //    std::cout << "Request: [" << transformToWorld->data.orientation.arr[0] << " " << transformToWorld->data.orientation.arr[1]  << " " << transformToWorld->data.orientation.arr[2] <<" " << transformToWorld->data.orientation.arr[3] << "]" << std::endl;
        //}

        


        //std::cout << "Input R2W Pose" << std::endl;
        //printPose(*transformToWorld.get());

        //std::cout << " Time stamps: " << std::endl;
        //std::cout << "[PARENT] " << transformToWorld->metadata.parentTime.microseconds << ", " << transformToWorld->metadata.parentTime.usecPerSec << std::endl;
        //std::cout << "[CHILD] " << transformToWorld->metadata.childTime.microseconds << ", " << transformToWorld->metadata.childTime.usecPerSec << std::endl;
        //std::cout << "Frames: " << std::endl;
        //std::cout << "[PARENT] " << transformToWorld->metadata.parentFrameId.arr << std::endl;
        //std::cout << "[CHILD] " << transformToWorld->metadata.childFrameId.arr << std::endl;

        ////////////////////////////////////////////
        return true;
    }

    void cloudTransformASN1::getPoseFromPointCloud()
    {
        transformToWorld.reset(new asn1SccTransformWithCovariance(pcMsgInput->metadata.pose_fixedFrame_robotFrame));
        std::cout << "Input R2W Pose" << std::endl;
        printPose(*transformToWorld.get());
    }

    
    bool cloudTransformASN1::update_transform(/*pointCloudMsg,tfSensor2World*/)
    {
        //Update Rotation from sensor to robot from msg
        rotationSensor2Robot = Eigen::Quaterniond(pcMsgInput->metadata.pose_robotFrame_sensorFrame.data.orientation.arr[3],pcMsgInput->metadata.pose_robotFrame_sensorFrame.data.orientation.arr[0],pcMsgInput->metadata.pose_robotFrame_sensorFrame.data.orientation.arr[1],pcMsgInput->metadata.pose_robotFrame_sensorFrame.data.orientation.arr[2]);

        //Convert to rotation matrix
        transformSensor2Robot.block<3,3>(0,0) = rotationSensor2Robot.normalized().toRotationMatrix();

        //Convert to homogeneous transformation
        transformSensor2Robot(0,3) = pcMsgInput->metadata.pose_robotFrame_sensorFrame.data.translation.arr[0];
        transformSensor2Robot(1,3) = pcMsgInput->metadata.pose_robotFrame_sensorFrame.data.translation.arr[1];
        transformSensor2Robot(2,3) = pcMsgInput->metadata.pose_robotFrame_sensorFrame.data.translation.arr[2];
        transformSensor2Robot(3,3) = 1;

        //Update Rotation from robot to world from msg
        rotationRobot2World = Eigen::Quaterniond(transformToWorld->data.orientation.arr[3],transformToWorld->data.orientation.arr[0],transformToWorld->data.orientation.arr[1],transformToWorld->data.orientation.arr[2]);
        
        //Convert to rotation matrix
        transformRobot2World.block<3,3>(0,0) = rotationRobot2World.normalized().toRotationMatrix();

        for (int i = 0; i < 3; i++)
        {
            transformRobot2World(3,i) = 0;
            transformSensor2Robot(3,i) = 0;
        }

        //Convert to homogeneous transformation
        transformRobot2World(0,3) = transformToWorld->data.translation.arr[0];
        transformRobot2World(1,3) = transformToWorld->data.translation.arr[1];
        transformRobot2World(2,3) = transformToWorld->data.translation.arr[2];
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
        pointCloud.resize(pcMsgInput->data.points.nCount);
        auto it = pointCloud.begin();
        for (int i=0; i < pcMsgInput->data.points.nCount; i++)
        {
            (*it)[0] = pcMsgInput->data.points.arr[i].arr[0]; 
            (*it)[1] = pcMsgInput->data.points.arr[i].arr[1]; 
            (*it)[2] = pcMsgInput->data.points.arr[i].arr[2]; 
            it++;
        }

        timeStamp = pcMsgInput->metadata.timeStamp.microseconds;
        return true;
    }

    bool cloudTransformASN1::update_outputMsg(/*pcMsgOutput,pointCloud,tfSensor2World*/)
    {
        // Change frameId
        std::string frameMsg = fixedFrame;
        toASN1SCC(frameMsg,pcMsgOutput->metadata.frameId);

        /* Copy all other informations */
        pcMsgOutput->metadata.msgVersion = pcMsgInput->metadata.msgVersion;
        pcMsgOutput->metadata.sensorId = pcMsgInput->metadata.sensorId;
        pcMsgOutput->metadata.timeStamp = pcMsgInput->metadata.timeStamp;
        
        pcMsgOutput->metadata.isRegistered = pcMsgInput->metadata.isRegistered;
        pcMsgOutput->metadata.isOrdered = pcMsgInput->metadata.isOrdered;
        pcMsgOutput->metadata.height = pcMsgInput->metadata.height;
        pcMsgOutput->metadata.width = pcMsgInput->metadata.width;

        pcMsgOutput->metadata.pose_robotFrame_sensorFrame = pcMsgInput->metadata.pose_robotFrame_sensorFrame;
        //pcMsgOutput->metadata.pose_fixedFrame_robotFrame = pcMsgInput->metadata.pose_fixedFrame_robotFrame;


        pcMsgOutput->data.colors = pcMsgInput->data.colors;
        pcMsgOutput->data.intensity = pcMsgInput->data.intensity;

        //Update the pose
        pcMsgOutput->metadata.pose_fixedFrame_robotFrame = *transformToWorld.get();

        //Update the point cloud

        auto it =pointCloud.begin();
        for (int i=0; i< pcMsgInput->data.points.nCount; i++)
        {
            pcMsgOutput->data.points.arr[i].arr[0] = (*it)[0];
            pcMsgOutput->data.points.arr[i].arr[1] = (*it)[1];
            pcMsgOutput->data.points.arr[i].arr[2] = (*it)[2];
            it++;
        }
        pcMsgOutput->data.intensity.nCount = pcMsgInput->data.points.nCount;
        pcMsgOutput->data.colors.nCount = pcMsgInput->data.colors.nCount;
        pcMsgOutput->data.points.nCount = pcMsgInput->data.points.nCount;

        //std::cout << "========================= " << std::endl;
        //std::cout << "pcMsgOutput: pose R2W" << std::endl;
        //printPose(pcMsgOutput->metadata.pose_fixedFrame_robotFrame);

        return true;
    }
    
    BitStream cloudTransformASN1::encode_message(/*pcMsgOutput*/)
    {
        int errorCode;
        BitStream b;

        BitStream_Init(&b,perBuffer,asn1SccPointcloud_REQUIRED_BYTES_FOR_ENCODING);
        if (!asn1SccPointcloud_Encode(pcMsgOutput.get(),&b,&errorCode,TRUE))
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

        asn1SccTransformWithCovariance request;
        toASN1SCC(fixedFrame,request.metadata.parentFrameId);
        request.metadata.childFrameId = pcMsgInput->metadata.pose_robotFrame_sensorFrame.metadata.parentFrameId;
        request.metadata.producerId = pcMsgInput->metadata.pose_robotFrame_sensorFrame.metadata.producerId;

        std::string childFrame("RoverBodyFrame");
        toASN1SCC(childFrame,request.metadata.childFrameId);
        request.metadata.msgVersion = transformWithCovariance_version;
        request.metadata.parentTime = pcMsgInput->metadata.timeStamp;
        request.metadata.childTime = pcMsgInput->metadata.timeStamp;
        request.data = pcMsgInput->metadata.pose_robotFrame_sensorFrame.data;

        /* Encoding */

        BitStream_Init(&b,perBufferPose,asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);

        if (!asn1SccTransformWithCovariance_Encode(&request,&b,&errorCode,TRUE))
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
        std::cout << "======== Header frameId: " << pcMsgInput->metadata.frameId.arr << std::endl;
        std::cout << "======== nCount frameId: " << pcMsgInput->metadata.frameId.nCount << std::endl;
        std::cout << "======== timeStamp: [" << pcMsgInput->metadata.timeStamp.microseconds << ", " << pcMsgInput->metadata.timeStamp.usecPerSec << "]" <<  std::endl;

        std::cout << "==== On the pointCloud: " << std::endl;
        std::cout << "======== nCount pointCloud: " << pcMsgInput->data.points.nCount << std::endl;
        std::cout << "======== nCount colors: " << pcMsgInput->data.colors.nCount << std::endl;
        std::cout << "======== nCount intensity: " << pcMsgInput->data.intensity.nCount << std::endl;


        std::cout << "==== On the poses [Frame ids]: " << std::endl;
        std::cout << "======== World->robot [PARENT]: " <<  pcMsgInput->metadata.pose_fixedFrame_robotFrame.metadata.parentFrameId.arr << ",nCount: " << pcMsgInput->metadata.pose_fixedFrame_robotFrame.metadata.parentFrameId.nCount <<std::endl;
        std::cout << "======== World->robot[CHILD]: " <<  pcMsgInput->metadata.pose_fixedFrame_robotFrame.metadata.childFrameId.arr << ",nCount: " << pcMsgInput->metadata.pose_fixedFrame_robotFrame.metadata.childFrameId.nCount << std::endl;
        std::cout << "======== Robot->sensor[PARENT]: " << pcMsgInput->metadata.pose_robotFrame_sensorFrame.metadata.parentFrameId.arr << ", nCount: " << pcMsgInput->metadata.pose_robotFrame_sensorFrame.metadata.parentFrameId.nCount << std::endl;
        std::cout << "======== Robot->sensor[CHILD]: " << pcMsgInput->metadata.pose_robotFrame_sensorFrame.metadata.childFrameId.arr << ", nCount: " << pcMsgInput->metadata.pose_robotFrame_sensorFrame.metadata.childFrameId.nCount << std::endl;


        std::cout << "==== On the poses[Times]: " << std::endl;
        std::cout << "======== World -> Robot[PARENT]: " << pcMsgInput->metadata.pose_fixedFrame_robotFrame.metadata.parentTime.microseconds << ", " << pcMsgInput->metadata.pose_fixedFrame_robotFrame.metadata.parentTime.usecPerSec << std::endl;
        std::cout << "======== World -> Robot[CHILD]: " << pcMsgInput->metadata.pose_fixedFrame_robotFrame.metadata.childTime.microseconds << ", " << pcMsgInput->metadata.pose_fixedFrame_robotFrame.metadata.childTime.usecPerSec << std::endl;
        std::cout << "======== Robot -> sensor [PARENT]: " << pcMsgInput->metadata.pose_robotFrame_sensorFrame.metadata.parentTime.microseconds << ", " << pcMsgInput->metadata.pose_robotFrame_sensorFrame.metadata.parentTime.usecPerSec << std::endl;
        std::cout << "======== Robot -> sensor [CHILD]: " << pcMsgInput->metadata.pose_robotFrame_sensorFrame.metadata.childTime.microseconds << ", " << pcMsgInput->metadata.pose_robotFrame_sensorFrame.metadata.childTime.usecPerSec << std::endl;
    }
    
    void cloudTransformASN1::print_outputMsg()
    {
        std::cout << "pcMsgOutput Metadata: " << std::endl;
        std::cout << "==== General Metadata: " << std::endl;
        std::cout << "======== Header frameId: " << pcMsgOutput->metadata.frameId.arr << std::endl;
        std::cout << "======== nCount frameId: " << pcMsgOutput->metadata.frameId.nCount << std::endl;
        std::cout << "======== timeStamp: [" << pcMsgOutput->metadata.timeStamp.microseconds << ", " << pcMsgOutput->metadata.timeStamp.usecPerSec << "]" <<  std::endl;

        std::cout << "==== On the pointCloud: " << std::endl;
        std::cout << "======== nCount pointCloud: " << pcMsgOutput->data.points.nCount << std::endl;
        std::cout << "======== nCount colors: " << pcMsgOutput->data.colors.nCount << std::endl;
        std::cout << "======== nCount intensity: " << pcMsgOutput->data.intensity.nCount << std::endl;


        std::cout << "==== On the poses [Frame ids]: " << std::endl;
        std::cout << "======== World->robot [PARENT]: " <<  pcMsgOutput->metadata.pose_fixedFrame_robotFrame.metadata.parentFrameId.arr << ",nCount: " << pcMsgOutput->metadata.pose_fixedFrame_robotFrame.metadata.parentFrameId.nCount <<std::endl;
        std::cout << "======== World->robot[CHILD]: " <<  pcMsgOutput->metadata.pose_fixedFrame_robotFrame.metadata.childFrameId.arr << ",nCount: " << pcMsgOutput->metadata.pose_fixedFrame_robotFrame.metadata.childFrameId.nCount << std::endl;
        std::cout << "======== Robot->sensor[PARENT]: " << pcMsgOutput->metadata.pose_robotFrame_sensorFrame.metadata.parentFrameId.arr << ", nCount: " << pcMsgOutput->metadata.pose_robotFrame_sensorFrame.metadata.parentFrameId.nCount << std::endl;
        std::cout << "======== Robot->sensor[CHILD]: " << pcMsgOutput->metadata.pose_robotFrame_sensorFrame.metadata.childFrameId.arr << ", nCount: " << pcMsgOutput->metadata.pose_robotFrame_sensorFrame.metadata.childFrameId.nCount << std::endl;


        std::cout << "==== On the poses[Times]: " << std::endl;
        std::cout << "======== World -> Robot[PARENT]: " << pcMsgOutput->metadata.pose_fixedFrame_robotFrame.metadata.parentTime.microseconds << ", " << pcMsgOutput->metadata.pose_fixedFrame_robotFrame.metadata.parentTime.usecPerSec << std::endl;
        std::cout << "======== World -> Robot[CHILD]: " << pcMsgOutput->metadata.pose_fixedFrame_robotFrame.metadata.childTime.microseconds << ", " << pcMsgOutput->metadata.pose_fixedFrame_robotFrame.metadata.childTime.usecPerSec << std::endl;
        std::cout << "======== Robot -> sensor [PARENT]: " << pcMsgOutput->metadata.pose_robotFrame_sensorFrame.metadata.parentTime.microseconds << ", " << pcMsgOutput->metadata.pose_robotFrame_sensorFrame.metadata.parentTime.usecPerSec << std::endl;
        std::cout << "======== Robot -> sensor [CHILD]: " << pcMsgOutput->metadata.pose_robotFrame_sensorFrame.metadata.childTime.microseconds << ", " << pcMsgOutput->metadata.pose_robotFrame_sensorFrame.metadata.childTime.usecPerSec << std::endl;
    }
    
    void cloudTransformASN1::printPose(asn1SccTransformWithCovariance pose)
    {
        std::cout << "== Position" << std::endl;
        std::cout << "==== [";
        std::cout << pose.data.translation.arr[0] << ", " << pose.data.translation.arr[1] << ", " << pose.data.translation.arr[2];
        std::cout << "]" << std::endl;
        std::cout << "== Orientation " << std::endl;
        std::cout << "==== [";
        std::cout << "w = " << pose.data.orientation.arr[3];
        std::cout << ", x = " << pose.data.orientation.arr[0];
        std::cout << ", y = " << pose.data.orientation.arr[1];
        std::cout << ", z = " << pose.data.orientation.arr[2];
        std::cout << "]" << std::endl;
    }
};
