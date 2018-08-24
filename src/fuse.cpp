#ifndef __FUSE_CPP__
#define __FUSE_CPP__

#include <infuse_dem_building/fuse.hpp>

namespace dem_building
{
    mapFuser::mapFuser()
    {
    }
    
    mapFuser::~mapFuser()
    {
        clean_up();
    }

    void mapFuser::clean_up()
    {
    }



    void mapFuser::init(int mapWidth, int mapHeight)
    {
        current = {{0,0}};
        width = mapWidth;
        height = mapHeight;

        sw = width / 3;
        sh = height / 3;

        isInit = true;

        fusedMap.resize(width * height);
        roverMap.resize(width * height);
        gndinter.resize( width * height );

        use_swap = true;

        tile_load(0, 0);
        tile_load(0, 1);
        tile_load(0, 2);
        tile_load(1, 0);
        tile_load(1, 1);
        tile_load(1, 2);
        tile_load(2, 0);
        tile_load(2, 1);
        tile_load(2, 2);
    }


    void mapFuser::tile_load(int sx, int sy)
    {
        std::string filepath = tilepath(current[0] + sx, current[1] + sy);

        if ( ! file_exists( filepath ) )
        {
            return; // no file to load
        }

        tile.load(filepath);
        assert( tile.bands.size() == MAP_NAMES.size() );
        assert( tile.bands[0].size() == (unsigned int) (sw * sh) );
        long diff = time_base - std::stol(tile.get_meta("TIME", "0"));
        size_t idx = 0, eoi = 0;
        for (auto it  = fusedMap.begin() + sw * sx + sh * width * sy,
                end = it + sh * width; it < end; it += width - sw)
        {
            for (eoi += sw; idx < eoi; idx++) 
            {
                // tile to map
                (*it)[N_POINTS] = tile.bands[N_POINTS][idx];
                (*it)[Z_MAX]    = tile.bands[Z_MAX][idx];
                (*it)[Z_MIN]    = tile.bands[Z_MIN][idx];
                (*it)[Z_MEAN]   = tile.bands[Z_MEAN][idx];
                (*it)[VARIANCE] = tile.bands[VARIANCE][idx];
                (*it)[TIME]     = tile.bands[TIME][idx];
                (*it)[DIST_SQ]  = tile.bands[DIST_SQ][idx];
                if ( diff and (*it)[N_POINTS] > 0.9 )
                {
                    (*it)[TIME] -= diff;
                }
                it++;
            }
        }
    }

    void mapFuser::tile_save(int sx, int sy /*, fusedMap, current*/) const
    {
        tile.names = MAP_NAMES;
        tile.set_size(N_RASTER, sw, sh);
        size_t idx = 0, eoi = 0;
        for (auto it  = fusedMap.begin() + sw * sx + sh * width * sy,
                end = it + sh * width; it < end; it += width - sw)
        {
            for (eoi += sw; idx < eoi; idx++) 
            {
                // map to tile
                tile.bands[N_POINTS][idx] = (*it)[N_POINTS];
                tile.bands[Z_MAX][idx]    = (*it)[Z_MAX];
                tile.bands[Z_MIN][idx]    = (*it)[Z_MIN];
                tile.bands[Z_MEAN][idx]   = (*it)[Z_MEAN];
                tile.bands[VARIANCE][idx] = (*it)[VARIANCE];
                tile.bands[TIME][idx]     = (*it)[TIME];
                tile.bands[DIST_SQ][idx]  = (*it)[DIST_SQ];
                it++;
            }
        }
        if (any_gt_zero(tile.bands[N_POINTS])) // dont save empty tiles
        { 
            // update map transform used for merging the pointcloud
            tile.set_transform(xOrigin, yOrigin,scale,scale);
            tile.save(tilepath(current[0] + sx, current[1] + sy) );
        }
    }

    bool mapFuser::load_data(/*fusedMap*/)
    {   
        int dx = newTile[0] - current[0];
        int dy = newTile[1] - current[1];
        cell_info_t zeros{}; // value-initialization w/empty initializer
        std::fill(gndinter.begin(), gndinter.end(), zeros);

        if (dx == 0 && dy == 0)
        {
            std::cout << "On the right tile" << std::endl;
            return true;
        }

        // FIRST SAVE THE RIGHT DATA

        if (dx == -1) // SAVE EAST
        {
            // save EAST 1/3 maplets [ 1,-1], [ 1, 0], [ 1, 1]
            tile_save(2, 0);
            tile_save(2, 1);
            tile_save(2, 2);
            if (dy == -1) 
            {
                // save south
                tile_save(0, 2);
                tile_save(1, 2);
            } 
            else if (dy == 1) 
            {
                // save NORTH
                tile_save(0, 0);
                tile_save(1, 0);
            }
            for (auto it = fusedMap.begin(); it < fusedMap.end(); it += width) 
            {
                std::copy_backward(it, it + 2 * sw, it + width);
                std::fill(it, it + sw, zeros);
            }
        }
        else if (dx == 1) 
        {
            // save WEST 1/3 maplets [-1,-1], [-1, 0], [-1, 1]
            tile_save(0, 0);
            tile_save(0, 1);
            tile_save(0, 2);
            if (dy == -1) 
            {
                // save SOUTH
                tile_save(1, 2);
                tile_save(2, 2);
            } 
            else if (dy == 1) 
            {
                // save NORTH
                tile_save(1, 0);
                tile_save(2, 0);
            }
            // move the map to the EAST
            for (auto it = fusedMap.begin(); it < fusedMap.end(); it += width) 
            {
                std::copy(it + sw, it + width, it);
                // reset(it + 2 * sw, it + width);
                std::fill(it + 2 * sw, it + width, zeros);
            }
        } 
        else if (dy == -1) 
        {
            // save SOUTH
            tile_save(0, 2);
            tile_save(1, 2);
            tile_save(2, 2);
        } 
        else if (dy == 1) 
        {
            // save NORTH
            tile_save(0, 0);
            tile_save(1, 0);
            tile_save(2, 0);
        }
        if (dy == -1) 
        {
            std::copy_backward(fusedMap.begin(), fusedMap.end() - sh * width,
                    fusedMap.end());
            // reset(fusedMap.begin(), fusedMap.begin() + sh * width);
            std::fill(fusedMap.begin(), fusedMap.begin() + sh * width - 1, zeros);
        } 
        else if (dy == 1) 
        {
            std::copy(fusedMap.begin() + sh * width, fusedMap.end(), fusedMap.begin());
            // reset(fusedMap.end() - sh * width, fusedMap.end());
            std::fill(fusedMap.end() - sh * width, fusedMap.end(), zeros);
        }

        // THEN UPDATE THE TILE WE'RE ON

        current[0] = newTile[0];
        current[1] = newTile[1];

        // THEN LOAD THE RIGHT DATA
        if (dx == -1) 
        {
            // load WEST maplets
            tile_load(0, 0);
            tile_load(0, 1);
            tile_load(0, 2);
            if (dy == -1) 
            {
                // load NORTH
                tile_load(1, 0);
                tile_load(2, 0);
            } 
            else if (dy == 1) 
            {
                // load SOUTH
                tile_load(1, 2);
                tile_load(2, 2);
            }
        }
        else if (dx == 1) 
        {
            // load EAST maplets
            tile_load(2, 0);
            tile_load(2, 1);
            tile_load(2, 2);
            if (dy == -1) 
            {
                // load NORTH
                tile_load(0, 0);
                tile_load(1, 0);
            } else if (dy == 1) 
            {
                // load SOUTH
                tile_load(0, 2);
                tile_load(1, 2);
            }
        }
        else if (dy == -1) 
        {
            // load NORTH
            tile_load(0, 0);
            tile_load(1, 0);
            tile_load(2, 0);
        } 
        else if (dy == 1) 
        {
            // load SOUTH
            tile_load(0, 2);
            tile_load(1, 2);
            tile_load(2, 2);
        }
        return true;
    }

    bool mapFuser::fuse(/*fusedMap,roverMap*/)
    {
        bool is_vertical;
        size_t index = 0;
        //float time_ref = get_reference_time();
        auto it = fusedMap.begin();

        for (auto& dyninfo : roverMap) 
        {
            if ( dyninfo[N_POINTS] > 0 && (
                        (*it)[N_POINTS] < 1 || (*it)[DIST_SQ] - dyninfo[DIST_SQ] > -4 ) ) 
            {
                /* compute the real variance (according to Knuth's bible) */
                if (dyninfo[N_POINTS] > 2)
                {
                    dyninfo[VARIANCE] /= dyninfo[N_POINTS] - 1;
                }

                is_vertical = dyninfo[VARIANCE] > variance_threshold;

                if ( (*it)[N_POINTS] < 1 || ( dyninfo[N_POINTS] > 2 &&
                            (*it)[DIST_SQ] - dyninfo[DIST_SQ] > 4 ) ) 
                {
                    // init
                    *it = dyninfo;
                } 
                else if (use_swap) 
                {
                    if ( is_vertical == ( (*it)[VARIANCE] > variance_threshold) ) 
                    {
                        // same state
                        // if the cells are flat and differ more than 10cm, swap
                        if (!is_vertical && (( (*it)[Z_MEAN] - dyninfo[Z_MEAN] ) > 0.1 )) 
                        {
                            gndinter[index] = *it;
                            *it = dyninfo;
                            // TODO (*it)[DYNAMIC] += 1.0;
                        } 
                        else 
                        {
                            merge(*it, dyninfo);
                        }
                    } 
                    else if ( is_vertical ) 
                    {
                        // was flat, backup the cell in ground swap
                        gndinter[index] = *it;
                        *it = dyninfo;
                        // TODO (*it)[DYNAMIC] += 1.0;
                    }
                    else 
                    {
                        // was vertical, revert ground and merge
                        *it = gndinter[index];
                        merge(*it, dyninfo);
                        // TODO (*it)[DYNAMIC] += 1.0;
                        // TODO gndinter[index] = zeros; ???
                    }
                } 
                else 
                {
                    merge(*it, dyninfo);
                }
                //(*it)[TIME] = time_ref;
            }

            it++;
            index++;
        }
        return true;
    }

    void mapFuser::merge(cell_info_t& dst, const cell_info_t& src) const 
    {
        if ( dst[N_POINTS] < 1 ) 
        {
            dst = src;
            return;
        }
        float z_mean, new_n_pts;

        new_n_pts = src[N_POINTS] + dst[N_POINTS];
        z_mean = dst[Z_MEAN];

        if (dst[Z_MAX] < src[Z_MAX])
        {
            dst[Z_MAX] = src[Z_MAX];
        }
        if (dst[Z_MIN] > src[Z_MIN])
        {
            dst[Z_MIN] = src[Z_MIN];
        }

        dst[Z_MEAN] = ( (z_mean * dst[N_POINTS]) + (src[Z_MEAN] * src[N_POINTS]) )
            / new_n_pts;
        // compute the global variance
        dst[VARIANCE] = ( src[VARIANCE] * (src[N_POINTS] - 1)
                + dst[VARIANCE] * (dst[N_POINTS] - 1)
                ) / (new_n_pts - 1);
        dst[N_POINTS] = new_n_pts;
    }
}
#endif
