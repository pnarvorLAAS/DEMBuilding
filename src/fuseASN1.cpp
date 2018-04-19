#include <atlaas/fuseASN1.hpp>

namespace atlaas
{
    mapFuserASN1::mapFuserASN1()
    {
        demMsgInput = new DigitalElevationMap;
        demRasterOutput = new DigitalElevationRaster;
        perBuffer = (byte*) malloc(sizeof(byte)*DigitalElevationRaster_REQUIRED_BYTES_FOR_ENCODING);
    }

    mapFuserASN1::~mapFuserASN1()
    {
        clean_up();
    }

    void mapFuserASN1::clean_up()
    {
        free(perBuffer);
        delete demMsgInput;
        delete demRasterOutput;
    }
    
    bool mapFuserASN1::decode_message(BitStream msg)
    {
        int errorCode;
        if (!DigitalElevationMap_Decode(demMsgInput,&msg,&errorCode))
        {
            std::cerr << "[Decoding] failed, error code: " << errorCode <<  std::endl;
            return false;
        }

        return true;
    }
    
    bool mapFuserASN1::update_rovermap(/*demMsgInput,roverMap*/)
    {
        if (!isInit)
        {
            init(demMsgInput->nbCols,demMsgInput->nbLines);
        }
        /* The following MIGHT be added inside a separate helperfunction file*/

        /* Position of the current center tile */
        newTile[0] = demMsgInput->currentTile.arr[0];
        newTile[1] = demMsgInput->currentTile.arr[1];

        /* This line is dangerous : No checking over format, we just go all-in on the fact the the sender organized the data structure well before sending */

        memcpy(&roverMap[0],&demMsgInput->zValue.arr[0],width*height*N_RASTER*sizeof(float));

        xOrigin = demMsgInput->xOrigin;
        yOrigin = demMsgInput->yOrigin;
        scale = demMsgInput->scale;

        /* DEBUG */
        //for (int i  = 0; i < width*height; i++)
        //{
        //    if (roverMap[i][Z_MEAN] != 0)
        //    {
        //        std::cout << "Found non zero: " << roverMap[i][Z_MEAN] <<  std::endl;
        //        std::cout << "New tile : " << newTile[0] << ", " << newTile[1] << std::endl;
        //        return 1;
        //    }
        //}

        /* DEBUG */

        return true;
    }
    
    bool mapFuserASN1::update_outputMsg()
    {
        demRasterOutput->nbLines = demMsgInput->nbLines;
        demRasterOutput->nbCols = demMsgInput->nbCols;
        
        for (int i = 0; i < (int) (width*height); i++)
        {   
            demRasterOutput->zValue.arr[i]  = fusedMap[i][Z_MEAN];
        }
        demRasterOutput->zValue.nCount = width*height;
        return true;
    }
    
    BitStream mapFuserASN1::encode_message(/*demRasterOutput*/)
    {
        BitStream msg;
        int errorCode;

        BitStream_Init(&msg,perBuffer,DigitalElevationRaster_REQUIRED_BYTES_FOR_ENCODING);

        if (!DigitalElevationRaster_Encode(demRasterOutput,&msg,&errorCode,TRUE))
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
