#ifndef __RASTERIZATION_CPP__
#define __RASTERIZATION_CPP__

#include <atlaas/rasterization.hpp>

namespace atlaas{

    pcRasterizer::pcRasterizer()
    {
        perBuffer = (byte*) malloc(DigitalElevationMap_REQUIRED_BYTES_FOR_ENCODING*sizeof(byte));
    }

    pcRasterizer::~pcRasterizer()
    {
        free(perBuffer);
    }

    bool pcRasterizer::decode_message(BitStream msg)
    {
        BitStream b; /* Will serve to decode incoming bitstream msg */
        int errorCode;
        BitStream_AttachBuffer(&b,msg.buf,BitStream_GetLength(&msg));

        if (!PointCloudPoseStamped_Decode(&pcMsgInput,&b,&errorCode))
        {
            std::cerr << "[Decoding] failed, error code: " << errorCode <<  std::endl;
            return false;
        }
        return true;
    }
    
    bool pcRasterizer::update_transform(/*pcMsgInput,tfSensor2World*/)
    {
        //Update quaternion from msg
        q = Eigen::Quaterniond(pcMsgInput.pose.pose.orient.arr);
        //Convert to rotation matrix
        homoTrans.block<3,3>(0,0) = q.normalized().toRotationMatrix();
        homoTrans(0,3) = pcMsgInput.pose.pose.pos.arr[0];
        homoTrans(1,3) = pcMsgInput.pose.pose.pos.arr[1];
        homoTrans(2,3) = pcMsgInput.pose.pose.pos.arr[2];

        for (int i=0; i<4;i++)
        {
            for (int j=0; j<4;j++)
            {
                tfSensor2World[i*4 + j] = homoTrans(i,j);
            }
        }
        return true;
    }
    
    bool pcRasterizer::update_pointCloud(/*pointCloudMsg,pointCloud*/)
    {
        pointCloud.resize(pcMsgInput.pointCloudData.nCount);
        auto it = pointCloud.begin();
        for (int i=0; i < pcMsgInput.pointCloudData.nCount; i++)
        {
            (*it)[0] = pcMsgInput.pointCloudData.arr[i].arr[0]; 
            (*it)[1] = pcMsgInput.pointCloudData.arr[i].arr[1]; 
            (*it)[2] = pcMsgInput.pointCloudData.arr[i].arr[2]; 
            (*it)[3] = pcMsgInput.pointCloudData.arr[i].arr[3]; 
        }
        return true;
    }



    void pcRasterizer::init(double size_x,double size_y, double scale,
            double custom_x, double custom_y, double custom_z,
            int utm_zone, bool utm_north=true)
    {
        //Size of the cartesian grid
        width  = std::ceil(size_x / scale); 
        height = std::ceil(size_y / scale);

        scaleMap = scale;

        //Metadata to know what tiles we need
        meta.set_size(width, height); // does not change the container
        meta.set_transform(custom_x, custom_y, scale, -scale);
        meta.set_utm(utm_zone, utm_north);
        meta.set_custom_origin(custom_x, custom_y, custom_z);

        //Set initial time
        set_time_base(milliseconds_since_epoch());

        //Current tile parameters
        current = {{0,0}};
        sw = width  / 3; // tile-width
        sh = height / 3; // tile-height

        //Resize RoverMap
        dyninter.resize( width * height );
    }
    
    void pcRasterizer::set_time_base(uint64_t base) {
        time_base = base;
        meta.metadata["TIME"] = std::to_string(time_base);
    }

    bool pcRasterizer::rasterize(/*pointCloud*/)
    {
        size_t index;
        float z_mean, n_pts, new_z;
        // merge point-cloud in internal structure
        for (const auto& point : pointCloud) {
            index = meta.index_custom(point[0], point[1]);
            if (index >= dyninter.size() )
                continue; // point is outside the map

            auto& info = dyninter[ index ];
            new_z = point[2];
            n_pts = info[N_POINTS];

            if (n_pts < 1) {
                info[N_POINTS] = 1;
                info[Z_MAX]  = new_z;
                info[Z_MIN]  = new_z;
                info[Z_MEAN] = new_z;
                info[VARIANCE] = 0;
                info[DIST_SQ] = distance_sq(sensor_xy, {{point[0], point[1]}});
            } else {
                z_mean = info[Z_MEAN];
                // increment N_POINTS
                info[N_POINTS]++;
                // update Z_MAX
                if (new_z > info[Z_MAX])
                    info[Z_MAX] = new_z;
                // update Z_MIN
                if (new_z < info[Z_MIN])
                    info[Z_MIN] = new_z;

                /* Incremental mean and variance updates (according to Knuth's bible,
                   Vol. 2, section 4.2.2). The actual variance will later be divided
                   by the number of samples plus 1. */
                info[Z_MEAN]    = (z_mean * n_pts + new_z) / info[N_POINTS];
                info[VARIANCE] += (new_z - z_mean) * (new_z - info[Z_MEAN]);
            }
        }
        return true;
    }

    void pcRasterizer::do_slide()
    {
        while (slide());
    }

    bool pcRasterizer::slide(/*sensor_xy, meta*/)
    {
        const point_xy_t& pixr = meta.point_custom2pix(sensor_xy[0], sensor_xy[1]);
        float cx = pixr[0] / width;
        float cy = pixr[1] / height;
        // check, slide, save, load
        if ( ( cx > 0.25 ) && ( cx < 0.75 ) &&
                ( cy > 0.25 ) && ( cy < 0.75 ) )
        {
            return false; // robot is in "center" square
        }
        int dx = (cx < 0.33) ? -1 : (cx > 0.66) ? 1 : 0; // W/E
        int dy = (cy < 0.33) ? -1 : (cy > 0.66) ? 1 : 0; // N/S
        current[0] += dx;
        current[1] += dy;
        const auto& utm = meta.point_pix2utm(sw * dx, sh * dy);
        // update map transform used for merging the pointcloud
        meta.set_transform(utm[0], utm[1], meta.get_scale_x(), meta.get_scale_y());
        return true;
    }
            
    bool pcRasterizer::update_outputMsg(/*demMsgOutput*/)
    {
        //Update current tile
        demMsgOutput.currentTile.arr[0] = current[0];
        demMsgOutput.currentTile.arr[1] = current[1];

        //Map size
        demMsgOutput.nbLines = height;
        demMsgOutput.nbCols = width;
        demMsgOutput.scale = scaleMap;
        demMsgOutput.zOrigin = 0.0;
        demMsgOutput.zScale = 1.0;
        memcpy(&demMsgOutput.zValue.arr[0],&dyninter[0],width*height*N_RASTER*sizeof(float));

        //xorigin,yorigin: coordinates of the top left pixel in world
		const gdalwrap::point_xy_t& custom_origin = meta.point_pix2custom(0, 0);
		demMsgOutput.xOrigin = custom_origin[0];
		demMsgOutput.yOrigin = custom_origin[1];

        return true;
    }

    BitStream pcRasterizer::encode_message(/*demMsgOutput*/)
    {
        BitStream msg;
        int errorCode;

        BitStream_Init(&msg,perBuffer,DigitalElevationMap_REQUIRED_BYTES_FOR_ENCODING);

        if (!DigitalElevationMap_Encode(&demMsgOutput,&msg,&errorCode,TRUE))
        {
            std::cout << "[Encoding] failed. Error code: " << errorCode << std::endl;
            exit(-1);
        }
        else
        {
            return msg;
        }
    }
}

#endif
