#ifndef __FUSE_HPP__
#define __FUSE_HPP__

#include <atlaas/common.hpp>
#include <DEM.h>

namespace atlaas{

    class mapFuser{
        private:
            DigitalElevationMap demInput;

            cells_info_t    fusedMap;
            cells_info_t    roverMap;
            map_id_t        currentTile;

            float           xOrigin;
            float           yOrigin;
            float           zOrigin;
            float           zScale;
            float           scale;

        public:
            
            bool decode_message(BitStream msg);
            bool load_data(/*fusedMap*/);

            bool fuse(/*fusedMap,roverMap*/);
            
        
    };
}

#endif
