#ifndef __FUSEASN1_HPP__
#define __FUSEASN1_HPP__

#include <atlaas/fuse.hpp>
#include <DEM.h>

namespace atlaas{

    class mapFuserASN1 : public mapFuser
    {
        private:
            DigitalElevationMap*    demMsgInput;
            DigitalElevationRaster* demRasterOutput;
            byte*                   perBuffer;

        public:
            mapFuserASN1();
            ~mapFuserASN1();

            void clean_up();

            bool decode_message(BitStream msg);
            bool update_rovermap(/*demMsgInput,roverMap*/);
            bool update_outputMsg(/*fusedMap,demMsgOutput*/);
            BitStream encode_message(/*demMsgOutput*/);
    };
    
};

#endif
