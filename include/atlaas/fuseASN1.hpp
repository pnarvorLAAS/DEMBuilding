#ifndef __FUSEASN1_HPP__
#define __FUSEASN1_HPP__

#include <atlaas/fuse.hpp>
#include <DEM.h>
#include <memory>

namespace atlaas{

    class mapFuserASN1 : public mapFuser
    {
        private:
            std::shared_ptr<DigitalElevationMap>    demMsgInput;
            std::shared_ptr<DigitalElevationRaster> demRasterOutput;
            byte*                   perBuffer;

        public:
            mapFuserASN1();
            ~mapFuserASN1();

            void clean_up();

            bool decode_message(BitStream msg);
            bool update_rovermap(/*demMsgInput,roverMap*/);
            bool update_outputMsg(/*fusedMap,demMsgOutput*/);
            BitStream encode_message(/*demMsgOutput*/);

            void setLocalMap(std::shared_ptr<DigitalElevationMap> ptr){demMsgInput = ptr;}
    };
    
};

#endif
