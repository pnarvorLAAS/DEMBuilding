#undef NDEBUG
#include <iostream>
#include <atlaas/pctransform.hpp>
#include <PointCloudPoseStamped.h>
#include <conversions/asn1scc_conversions.h>
#include <sys/time.h>

using namespace std;

void create_plan_pointCloud(int quarter,PointCloudPoseStamped &pc);
void create_sparse_pointCloud(PointCloudPoseStamped &pc);
void create_translation_transform(PoseStamped &pose);
void fill_header(Header &header);

int main(int argc, char* argv[])
{

    atlaas::cloudTransform test;
    return 0;
}

void fill_header(Header &header)
{
    std::string frame = "frame";
    T_String frameId;
    toASN1SCC(frame,frameId);

    header.frameId = frameId;
    header.msgVersion = 1;

    /* Set time with timeoftheday */
    timeval ts;
    gettimeofday(&ts,NULL);
    header.timeStamp.microseconds = ts.tv_sec;
    header.timeStamp.usecPerSec = ts.tv_usec;
}

void create_translation_transform(PoseStamped &pose)
{
    /* Header */

    fill_header(pose.header);

    /* ChildFrame */

    std::string childFrame = "frame2";
    T_String frameId;
    toASN1SCC(childFrame,frameId);
    pose.childFrameId = frameId;

    /* Pose */
    
        /* Position */

    pose.pose.pos.nCount = 3;
    pose.pose.pos.arr[0] = 0;
    pose.pose.pos.arr[1] = 0;
    pose.pose.pos.arr[2] = 2;

        /* Orientation */ 

    pose.pose.orient.nCount = 4;
    pose.pose.orient.arr[0] = 0;
    pose.pose.orient.arr[1] = 0;
    pose.pose.orient.arr[2] = 0;
    pose.pose.orient.arr[3] = 1;
    
}


