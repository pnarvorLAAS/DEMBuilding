#include <atlaas/rasterizationASN1.hpp>

namespace atlaas{
        
        pcRasterizerASN1::pcRasterizerASN1()
        {
            perBuffer = (byte*) malloc(DigitalElevationMap_REQUIRED_BYTES_FOR_ENCODING*sizeof(byte));
            perBufferRaster = (byte*)malloc(DigitalElevationRaster_REQUIRED_BYTES_FOR_ENCODING*sizeof(byte));
            pcMsgInput = new PointCloudPoseStamped;
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
            if (!PointCloudPoseStamped_Decode(pcMsgInput,&msg,&errorCode))
            {
                std::cerr << "[Decoding] failed, error code: " << errorCode <<  std::endl;
                return false;
            }
            return true;
        }

        bool pcRasterizerASN1::update_transform(/*pcMsgInput,tfSensor2World*/)
        {
            //Update quaternion from msg
            q = Eigen::Quaterniond(pcMsgInput->pose.pose.orient.arr);
            //Convert to rotation matrix
            homoTrans.block<3,3>(0,0) = q.normalized().toRotationMatrix();
            homoTrans(0,3) = pcMsgInput->pose.pose.pos.arr[0];
            homoTrans(1,3) = pcMsgInput->pose.pose.pos.arr[1];
            homoTrans(2,3) = pcMsgInput->pose.pose.pos.arr[2];
            homoTrans(3,3) = 1.0;


            //std::cout << "Transform:[  " << std::endl;
            for (int i=0; i<4;i++)
            {
                for (int j=0; j<4;j++)
                {
                    tfSensor2World[i*4 + j] = homoTrans(i,j);
                    //std::cout << tfSensor2World[i*4 + j] << ", " ;
                }
                //std::cout << std::endl;
            }
            //std::cout << "]" << std::endl;
            return true;
        }
    
        bool pcRasterizerASN1::update_pointCloud(/*pointCloudMsg,pointCloud*/)
        {
            pointCloud.resize(pcMsgInput->pointCloudData.nCount);
            auto it = pointCloud.begin();
            for (int i=0; i < pcMsgInput->pointCloudData.nCount; i++)
            {
                (*it)[0] = pcMsgInput->pointCloudData.arr[i].arr[0]; 
                (*it)[1] = pcMsgInput->pointCloudData.arr[i].arr[1]; 
                (*it)[2] = pcMsgInput->pointCloudData.arr[i].arr[2]; 
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
            demRasterMsgOutput->header = pcMsgInput->header;
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
