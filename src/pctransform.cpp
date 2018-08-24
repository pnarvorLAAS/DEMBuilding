#ifndef _PCTRANSFORM_CPP
#define _PCTRANSFORM_CPP

#include <infuse_dem_building/pctransform.hpp>
#include <iostream>

namespace dem_building{

    cloudTransform::cloudTransform()
    {
        std::cout << "cloudTransform created" << std::endl;
    }

    cloudTransform::~cloudTransform()
    {
        clean_up();
    }

    void cloudTransform::clean_up()
    {
        std::cout << "Cleaning up the cloud tranformer!" << std::endl;
    }

    bool cloudTransform::transform_pointCloud(/*pointCloud*/)
    {
        float x,y,z;
        for (auto& point : pointCloud) 
        {
            x = point[0];
            y = point[1];
            z = point[2];
            point[0] = (x * tfSensor2World[0]) + (y * tfSensor2World[1]) + (z * tfSensor2World[2])  + tfSensor2World[3];
            point[1] = (x * tfSensor2World[4]) + (y * tfSensor2World[5]) + (z * tfSensor2World[6])  + tfSensor2World[7];
            point[2] = (x * tfSensor2World[8]) + (y * tfSensor2World[9]) + (z * tfSensor2World[10]) + tfSensor2World[11];
        }
        return true;
    }
}

#endif
