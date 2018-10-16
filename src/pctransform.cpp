#ifndef _PCTRANSFORM_CPP
#define _PCTRANSFORM_CPP

#include <infuse_dem_building/pctransform.hpp>
#include <iostream>

#ifdef _WITH_PCL
    #include <infuse_dem_building/pcd.hpp>
#endif

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

    #ifdef _WITH_PCL

    void cloudTransform::save_pcd(const std::string& filename)
    {
        write_pcd(filename, pointCloud, tfSensor2World);
        std::ofstream logFile;
        logFile.open(filename.substr(0,filename.find_last_of("/") + 1),std::ofstream::app);
        if (logFile.is_open())
        {
            logFile << timeStamp << " " << filename << "\n";
        }
        logFile.close();

    }

    void cloudTransform::read_pcd(const std::string& filename)
    {
        read_pcd_file(filename, pointCloud, tfSensor2World);
    }

    #else
    
    void cloudTransform::save_pcd(const std::string& filename)
    {
        std::cout << "Support for PCL was not compiled, add compile option WITH_PCL to infuse_dem_building" << std::endl;

    }

    void cloudTransform::read_pcd(const std::string& filename)
    {
        std::cout << "Support for PCL was not compiled, add compile option WITH_PCL to infuse_dem_building" << std::endl;
    }

    #endif

}

#endif
