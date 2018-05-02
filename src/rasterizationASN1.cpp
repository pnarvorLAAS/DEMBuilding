#include <atlaas/rasterizationASN1.hpp>

namespace atlaas{
        
        pcRasterizerASN1::pcRasterizerASN1()
        {
            perBuffer = (byte*) malloc(DigitalElevationMap_REQUIRED_BYTES_FOR_ENCODING*sizeof(byte));
            perBufferRaster = (byte*)malloc(DigitalElevationRaster_REQUIRED_BYTES_FOR_ENCODING*sizeof(byte));
            pcMsgInput = new PointCloud_InFuse;
            demMsgOutput = new DigitalElevationMap; 
            demRasterMsgOutput = new DigitalElevationRaster;

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
            delete pcMsgInput;
            delete demMsgOutput;
            delete demRasterMsgOutput;
        }

        bool pcRasterizerASN1::decode_message(BitStream msg)
        {
            int errorCode;
            if (!PointCloud_InFuse_Decode(pcMsgInput,&msg,&errorCode))
            {
                std::cerr << "[Decoding] failed, error code: " << errorCode <<  std::endl;
                return false;
            }
            return true;
        }

        bool pcRasterizerASN1::update_transform(/*pcMsgInput,tfSensor2World*/)
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
            rotationRobot2World = Eigen::Quaterniond(pcMsgInput->pose_fixedFrame_robotFrame.transform.orientation.arr);
            
            //Convert to rotation matrix
            transformRobot2World.block<3,3>(0,0) = rotationRobot2World.normalized().toRotationMatrix();

            //Convert to homogeneous transformation
            transformRobot2World(0,3) = pcMsgInput->pose_robotFrame_sensorFrame.transform.translation.arr[0];
            transformRobot2World(1,3) = pcMsgInput->pose_robotFrame_sensorFrame.transform.translation.arr[1];
            transformRobot2World(2,3) = pcMsgInput->pose_robotFrame_sensorFrame.transform.translation.arr[2];

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
    
        bool pcRasterizerASN1::update_pointCloud(/*pointCloudMsg,pointCloud*/)
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
            //demRasterMsgOutput->header = pcMsgInput->header;
            demRasterMsgOutput->nbLines = height;
            demRasterMsgOutput->nbCols = width;
            demRasterMsgOutput->zValue.nCount = height*width;

            for (int i = 0; i < width*height; i++)
            {   
                demRasterMsgOutput->zValue.arr[i]  = dyninter[i][Z_MEAN];
            }
        }

        BitStream pcRasterizerASN1::encode_raster(/*demRasterMsgOutput*/)
        {
            BitStream msg;
            int errorCode;
            BitStream_Init(&msg,perBufferRaster,DigitalElevationRaster_REQUIRED_BYTES_FOR_ENCODING);

            if (!DigitalElevationRaster_Encode(demRasterMsgOutput,&msg,&errorCode,TRUE))
            {
                std::cout << "[Encoding Raster] failed. Error code: " << errorCode << std::endl;
                clean_up();
                exit(-1);
            }
            else
            {
                return msg;
            }
        }

        BitStream pcRasterizerASN1::encode_message(/*demMsgOutput*/)
        {
            BitStream msg;
            int errorCode;

            BitStream_Init(&msg,perBuffer,DigitalElevationMap_REQUIRED_BYTES_FOR_ENCODING);

            if (!DigitalElevationMap_Encode(demMsgOutput,&msg,&errorCode,TRUE))
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

};
