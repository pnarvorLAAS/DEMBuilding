#ifndef _PCTRANSFORM_HPP_
#define _PCTRANSFORM_HPP_

#include <PointCloudPoseStamped.h>
#include <atlaas/common.hpp>

namespace atlaas {

class cloudTransform
{
    private:
        points pointCloud;
        matrix tfSensor2World;
        PointCloudPoseStamped pointCloudMsg;

    public:
        bool decode_message(BitStream msg);
        bool update_transform(/*pointCloudMsg,tfSensor2World*/);
        bool update_pointCloud(/*pointCloudMsg,pointCloud*/);
        bool transform_pointCloud(/*pointCloud*/);
};

};


#endif
