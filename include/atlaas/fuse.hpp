#ifndef __FUSE_HPP__
#define __FUSE_HPP__

#include <atlaas/common.hpp>
#include <gdalwrap/gdal.hpp>
#include <cassert>
#include <cstring>
#include <DEM.h>

namespace atlaas{

    class mapFuser{
        private:
            DigitalElevationMap demMsgInput;
            DigitalElevationMap demMsgOutput;

            cells_info_t    fusedMap;
            cells_info_t    roverMap;
            map_id_t        current;
            map_id_t        newTile;

            mutable gdalwrap::gdal   tile;
            std::string     atlaas_path; //Where to store fused maps

            int             sw;
            int             sh;
            size_t          width;
            size_t          height;
            uint64_t        time_base;

            float           xOrigin;
            float           yOrigin;
            float           zOrigin;
            float           zScale;
            float           scale;

            bool            isInit = false;
            float        variance_threshold;
            cells_info_t gndinter; // ground info for vertical/flat unknown state
            bool use_swap;


        public:

            bool decode_message(BitStream msg);
            void init(int mapWidth, int mapHeight);
            bool update_rovermap(/*demMsgInput,roverMap*/);

            void tile_load(int sx, int sy /*, fusedMap, current */);
            void tile_save(int sx, int sy /*, fusedMap, current */) const;
            bool load_data(/*fusedMap*/);
            

            bool fuse(/*fusedMap,roverMap*/);
            void merge(cell_info_t& dst, const cell_info_t& src) const; 

            
            std::string tilepath(int x, int y) const 
            {
                std::ostringstream oss;
                oss << atlaas_path << "/atlaas." << x << "x" << y << ".tif";
                return oss.str();
            }

            bool update_outputMsg(/*fusedMap,demMsgOutput*/);
            BitStream encode_message(/*demMsgOutput*/);



    };
}

#endif
