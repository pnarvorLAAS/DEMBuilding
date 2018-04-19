#ifndef _PCTRANSFORM_HPP_
#define _PCTRANSFORM_HPP_

#include <atlaas/common.hpp>
#include <Eigen/Geometry>

namespace atlaas {

class cloudTransform
{
    protected:

        /* Inputs and outputs of the DFN */
        points                  pointCloud; 
        matrix                  tfSensor2World;

        /* Internal variables */
        
        Eigen::Quaterniond      q; // Store incoming quaternion into msg
        Eigen::Matrix4d         homoTrans; // Store homogenous transformation

        /* Test variabes */

        int                     cloudSize;



    public:
        cloudTransform();
        ~cloudTransform();

        virtual void clean_up();

        bool transform_pointCloud(/*pointCloud*/);

        /* Setters and getters for DEBUG */

        bool set_transform(matrix tfToSet)
        {
            tfSensor2World=tfToSet;
            return true;
        };

        matrix get_transform()
        {
            return tfSensor2World;
        };

        bool set_pointCloud(points pcToSet,int size)
        {
            pointCloud=pcToSet;
            cloudSize = size;
            return true;
        };

        points get_pointCloud()
        {
            return pointCloud;
        };

        int get_cloudSize()
        {
            return cloudSize;
        }
};

};


#endif
