#ifndef __FUSEASN1_HPP__
#define __FUSEASN1_HPP__

#include <infuse_dem_building/fuse.hpp>
#include <infuse_asn1_types/Map.h>
#include <infuse_asn1_types/MultiLayeredMap.h>
#include <memory>

namespace dem_building{

    class mapFuserASN1 : public mapFuser
    {
        private:
            std::shared_ptr<asn1SccMultiLayeredMap>    demMsgInput;
            std::shared_ptr<asn1SccMap> demRasterOutput;
            byte*                   perBuffer;

        public:
            mapFuserASN1();
            ~mapFuserASN1();

            void clean_up();

            bool decode_message(BitStream msg);
            bool update_rovermap(/*demMsgInput,roverMap*/);
            bool update_outputMsg(/*fusedMap,demMsgOutput*/);
            BitStream encode_message(/*demMsgOutput*/);

            void setLocalMap(std::shared_ptr<asn1SccMultiLayeredMap> ptr){demMsgInput = ptr;}
    };
    
};

#endif
