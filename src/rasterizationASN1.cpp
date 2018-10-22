#include <infuse_dem_building/rasterizationASN1.hpp>

namespace dem_building{
        
        pcRasterizerASN1::pcRasterizerASN1()
        {
            perBuffer = (byte*) malloc(asn1SccMultiLayeredMap_REQUIRED_BYTES_FOR_ENCODING*sizeof(byte));
            perBufferRaster = (byte*)malloc(asn1SccMap_REQUIRED_BYTES_FOR_ENCODING*sizeof(byte));
            pcMsgInput = std::make_shared<asn1SccPointcloud>();
            demMsgOutput = std::make_shared<asn1SccMultiLayeredMap>(); 
            demRasterMsgOutput = std::make_shared<asn1SccMap>();

        }

        pcRasterizerASN1::~pcRasterizerASN1()
        {
            clean_up();
        }

        void pcRasterizerASN1::clean_up()
        {
            std::cout << "Cleaning up the ASN1 Rasterizer" << std::endl;
            free(perBuffer);
            free(perBufferRaster);
        }

        bool pcRasterizerASN1::decode_message(BitStream msg)
        {
            int errorCode;
            if (!asn1SccPointcloud_Decode(pcMsgInput.get(),&msg,&errorCode))
            {
                std::cerr << "[Decoding] failed, error code: " << errorCode <<  std::endl;
                return false;
            }
            return true;
        }

        bool pcRasterizerASN1::update_transform(/*pcMsgInput,tfSensor2World*/)
        {


            // DEBUG
            //std::cout << "R2W RASTERIZER INPUT MSG" << std::endl;
            //printPose(pcMsgInput->metadata.pose_fixedFrame_robotFrame);
            //std::cout << "S2R RASTERIZER INPUT MSG" << std::endl;
            //printPose(pcMsgInput->metadata.pose_robotFrame_sensorFrame);
            // DEBUG

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
            rotationRobot2World = Eigen::Quaterniond(pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.orientation.arr[3],pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.orientation.arr[0],pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.orientation.arr[1],pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.orientation.arr[2]);
            
            //Convert to rotation matrix
            transformRobot2World.block<3,3>(0,0) = rotationRobot2World.normalized().toRotationMatrix();

            //Convert to homogeneous transformation
            transformRobot2World(0,3) = pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.translation.arr[0];
            transformRobot2World(1,3) = pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.translation.arr[1];
            transformRobot2World(2,3) = pcMsgInput->metadata.pose_fixedFrame_robotFrame.data.translation.arr[2];
            transformRobot2World(3,3) = 1;

             for (int i = 0; i < 3; i++)
             {
                 transformRobot2World(3,i) = 0;
                 transformSensor2Robot(3,i) = 0;
             }

            //Compute total transform

            transformSensor2World = transformRobot2World * transformSensor2Robot;

            // Print transform

            //Eigen::Quaterniond ori(transformSensor2World.block<3,3>(0,0));

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
                    //std::cout << tfSensor2World[i*4 + j] << ", " ;
                }
                //std::cout << std::endl;
            }
            //std::cout << "]" << std::endl;

            //Update sensor position

            sensor_xy[0] = transformSensor2World(0,3);
            sensor_xy[1] = transformSensor2World(1,3);

            return true;
        }
    
        bool pcRasterizerASN1::update_pointCloud(/*pointCloudMsg,pointCloud*/)
        {
            pointCloud.resize(pcMsgInput->data.points.nCount);
            auto it = pointCloud.begin();
            for (int i=0; i < pcMsgInput->data.points.nCount; i++)
            {
                (*it)[0] = pcMsgInput->data.points.arr[i].arr[0]; 
                (*it)[1] = pcMsgInput->data.points.arr[i].arr[1]; 
                (*it)[2] = pcMsgInput->data.points.arr[i].arr[2]; 
                (*it)[3] = pcMsgInput->data.intensity.arr[i];
                it++;
            }
            return true;
        }
    
        bool pcRasterizerASN1::update_outputMsg(/*demMsgOutput*/)
        {
            //Update current tile
            demMsgOutput->currentTile.arr[0] = current[0];
            demMsgOutput->currentTile.arr[1] = current[1];

            //Map size
            demMsgOutput->nbLines = height;
            demMsgOutput->nbCols = width;
            demMsgOutput->scale = scaleMap;
            demMsgOutput->zOrigin = 0.0;
            demMsgOutput->zScale = 1.0;
            memcpy(&demMsgOutput->zValue.arr[0],&dyninter[0],width*height*N_RASTER*sizeof(float));

            demMsgOutput->zValue.nCount = width*height*N_RASTER;
            demMsgOutput->state.nCount = 0;



            //xorigin,yorigin: coordinates of the top left pixel in world
	    	const gdalwrap::point_xy_t& custom_origin = meta.point_pix2custom(0, 0);
	    	demMsgOutput->xOrigin = custom_origin[0];
	    	demMsgOutput->yOrigin = custom_origin[1];

            return true;
        }

        bool pcRasterizerASN1::update_rasterMsg(/*demRasterMsgOutput,dyninter*/)
        {
            // Fill metadata

            demRasterMsgOutput->metadata.msgVersion = map_Version;
            demRasterMsgOutput->metadata.timeStamp = pcMsgInput->metadata.timeStamp;
            demRasterMsgOutput->metadata.type = asn1Sccmap_DEM;
            demRasterMsgOutput->metadata.scale  = 0.1;
            demRasterMsgOutput->metadata.pose_fixedFrame_mapFrame = pcMsgInput->metadata.pose_fixedFrame_robotFrame;

            // Fill 3DARRAY Metadata

            demRasterMsgOutput->data.msgVersion = array3D_Version;
            demRasterMsgOutput->data.rows = height;
            demRasterMsgOutput->data.cols = width;
            demRasterMsgOutput->data.channels = 1;
            demRasterMsgOutput->data.depth = asn1Sccdepth_32F;
            demRasterMsgOutput->data.rowSize = width*asn1Sccdepth_32F;

            demRasterMsgOutput->data.data.nCount = height*width*sizeof(float);

            for (int i = 0; i < width*height; i++)
            {   
                ((float*)demRasterMsgOutput->data.data.arr)[i]  = dyninter[i][Z_MEAN];
            }
        }

        BitStream pcRasterizerASN1::encode_raster(/*demRasterMsgOutput*/)
        {
            std::cout << "Entering the encode function" << std::endl;
            BitStream msg;
            int errorCode;
            BitStream_Init(&msg,perBufferRaster,asn1SccMap_REQUIRED_BYTES_FOR_ENCODING);

            if (!asn1SccMap_Encode(demRasterMsgOutput.get(),&msg,&errorCode,TRUE))
            {
                std::cout << "[Encoding Raster] failed. Error code: " << errorCode << std::endl;
                clean_up();
                exit(-1);
            }
            else
            {
                std::cout << "[Encoding Raster] Succeeded: count is " << msg.count << std::endl;
                return msg;
            }
            
        }

        BitStream pcRasterizerASN1::encode_message(/*demMsgOutput*/)
        {
            BitStream msg;
            int errorCode;

            BitStream_Init(&msg,perBuffer,asn1SccMultiLayeredMap_REQUIRED_BYTES_FOR_ENCODING);

            if (!asn1SccMultiLayeredMap_Encode(demMsgOutput.get(),&msg,&errorCode,TRUE))
            {
                std::cout << "[Encoding] failed. Error code: " << errorCode << std::endl;
                clean_up();
                exit(-1);
            }
            else
            {
                return msg;
            }

        }


        void pcRasterizerASN1::print_inputMsg()
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

    	void pcRasterizerASN1::printPose(asn1SccTransformWithCovariance pose)
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
