#include <infuse_dem_building/fuseASN1.hpp>

namespace dem_building
{
    mapFuserASN1::mapFuserASN1()
    {
        demMsgInput = std::make_shared<asn1SccMap>();
        demRasterOutput = std::make_shared<asn1SccMap>();
        perBuffer = (byte*) malloc(sizeof(byte)*asn1SccMap_REQUIRED_BYTES_FOR_ENCODING);
    }

    mapFuserASN1::~mapFuserASN1()
    {
        clean_up();
    }

    void mapFuserASN1::clean_up()
    {
        free(perBuffer);
    }
    
    bool mapFuserASN1::decode_message(BitStream msg)
    {
        int errorCode;
        if (!asn1SccMap_Decode(demMsgInput.get(),&msg,&errorCode))
        {
            std::cerr << "[Decoding] failed, error code: " << errorCode <<  std::endl;
            return false;
        }

        return true;
    }
    
    bool mapFuserASN1::update_rovermap(/*demMsgInput,roverMap*/)
    {

        /* Position of the current center tile */

        point_xy_t pixel = meta.point_utm2pix(demMsgInput->metadata.pose_fixedFrame_mapFrame.data.translation.arr[0],demMsgInput->metadata.pose_fixedFrame_mapFrame.data.translation.arr[1]);
        newTile[0] = current[0] + pixel[0]/sw;
        newTile[1] = current[1] + pixel[1]/sh ;

        /* This line is dangerous : No checking over format, we just go all-in on the fact the the sender organized the data structure well before sending */

        memcpy(&roverMap[0],&demMsgInput->data.data.arr[0],width*height*N_RASTER*sizeof(float));
        
        utmNewMap[0] = demMsgInput->metadata.pose_fixedFrame_mapFrame.data.translation.arr[0];
        utmNewMap[1] = demMsgInput->metadata.pose_fixedFrame_mapFrame.data.translation.arr[1],meta.get_scale_x(),meta.get_scale_y();

        std::cout << "New tile : " << newTile[0] << ", " << newTile[1] << std::endl;
        
        /* DEBUG */
        
        float int_min = std::numeric_limits<float>::max();
        float int_max = std::numeric_limits<float>::min();
        
        for (int i  = 0; i < width*height; i++)
        {
            if (roverMap[i][N_POINTS] > 0)
            {
                if (roverMap[i][INTENSITY] > int_max){ int_max = roverMap[i][INTENSITY];}
                if (roverMap[i][INTENSITY] < int_min){ int_min = roverMap[i][INTENSITY];}
            }
        }
        
        std::cout << "Min at arrival: " << int_min << std::endl;
        std::cout << "Max at arrival: " << int_max << std::endl;



        /* DEBUG */

        return true;
    }
    
    bool mapFuserASN1::update_outputMsg()
    {
        // Fill metadata

        float int_min = std::numeric_limits<float>::max();
        float int_max = std::numeric_limits<float>::min();

        demRasterOutput->metadata.msgVersion = map_Version;
        
        demRasterOutput->metadata.type = asn1Sccmap_DEM;
        demRasterOutput->metadata.scale  = 0.1;

        // Fill Pose

        demRasterOutput->metadata.pose_fixedFrame_mapFrame = demMsgInput->metadata.pose_fixedFrame_mapFrame;

        // Fill 3DARRAY Metadata

        demRasterOutput->data.msgVersion = array3D_Version;
        demRasterOutput->data.rows = height;
        demRasterOutput->data.cols = width;
        demRasterOutput->data.channels = 1;
        demRasterOutput->data.depth = asn1Sccdepth_32F;
        demRasterOutput->data.rowSize = width*asn1Sccdepth_32F;

        demRasterOutput->data.data.nCount = height*width*sizeof(float);

        for (int i = 0; i < width*height; i++)
        {   
            if (fusedMap[i][INTENSITY] > int_max){ int_max = fusedMap[i][INTENSITY];}
            if (fusedMap[i][INTENSITY] < int_min){ int_min = fusedMap[i][INTENSITY];}
            ((float*)demRasterOutput->data.data.arr)[i]  = fusedMap[i][INTENSITY];
        }

    }
    
    BitStream mapFuserASN1::encode_message(/*demRasterOutput*/)
    {
        BitStream msg;
        int errorCode;

        BitStream_Init(&msg,perBuffer,asn1SccMap_REQUIRED_BYTES_FOR_ENCODING);

        if (!asn1SccMap_Encode(demRasterOutput.get(),&msg,&errorCode,TRUE))
        {
            std::cout << "[Encoding] failed. Error code: " << errorCode << std::endl;
            exit(-1);
        }
        else
        {
            return msg;
        }
    }
};
