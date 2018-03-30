#undef NDEBUG
#include <iostream>
#include <atlaas/pctransform.hpp>
#include <atlaas/rasterization.hpp>
#include <atlaas/fuse.hpp>
#include <PointCloudPoseStamped.h>
#include <conversions/asn1_conversions.hpp>
#include <sys/time.h>

using namespace std;

void create_plan_point(int quarter,int xRange, int yRange, int zRange, double scale,double distanceFromRobot, PointCloudPoseStamped_pointCloudData &pc);
void create_translation_transform(PoseStamped &pose);
void fill_header(Header &header);

int main(int argc, char* argv[])
{

    /* Create point cloud message containing a plan */ 

    PointCloudPoseStamped* pc = new PointCloudPoseStamped();
    fill_header(pc->header);
    create_translation_transform(pc->pose);
    create_plan_point(0,20,20,10,0.1,5,pc->pointCloudData);
    cout << "data:  " << pc->pointCloudData.nCount << endl;


    /*Encode data to feed pointCloudTransform DFN */
    
    byte* perBuffer = (byte*) malloc(PointCloudPoseStamped_REQUIRED_BYTES_FOR_ENCODING); 
    BitStream b;
    int errorCode;

    BitStream_Init(&b,perBuffer,PointCloudPoseStamped_REQUIRED_BYTES_FOR_ENCODING);

    if (!PointCloudPoseStamped_Encode(pc,&b,&errorCode,TRUE))
    {
        cout << "Encoding of message failed error code: " << errorCode << endl;
    }

    /* Transform point cloud */
    
    atlaas::cloudTransform test;
    atlaas::pcRasterizer rast;
    test.decode_message(b);
    test.update_transform();
    test.update_pointCloud();
    test.transform_pointCloud();
    test.update_outputMsg();
    b = test.encode_message();
    rast.decode_message(b);

    PointCloudPoseStamped *pc2 = new PointCloudPoseStamped();


    /* Decode message */

    BitStream_AttachBuffer(&b,b.buf,BitStream_GetLength(&b));
    if (!PointCloudPoseStamped_Decode(pc2,&b,&errorCode))
    {
        cout << "Decoding failed error code: " << errorCode << endl;
    }

    cout << "Going to check " << pc2->pointCloudData.nCount << " points" << endl;
    for (int k = 0; k < pc2->pointCloudData.nCount; k++)
    {
        if (!(pc2->pointCloudData.arr[k].arr[1] - pc->pointCloudData.arr[k].arr[1] - 2.0 < 1e-6 &&  pc2->pointCloudData.arr[k].arr[1] - pc->pointCloudData.arr[k].arr[1] - 2.0 > -1e-6))
        {
            cout << "DIFFERENT at k =  " << k << endl;
            cout << "Original point cloud : " << pc->pointCloudData.arr[k].arr[2] << endl;
            cout << "Versus new point cloud : " << pc2->pointCloudData.arr[k].arr[2] << endl;
            cout << "Difference: " << pc2->pointCloudData.arr[k].arr[2] - pc->pointCloudData.arr[k].arr[2] - 2.0<< endl;
            break;
        }
    }
    
    
    

    /* Free allocated memory */
    
    delete pc;
    delete pc2;
    free(perBuffer);
    return 0;
}

void fill_header(Header &header)
{
    std::string frame = "frame";
    T_String frameId;
    toASN1SCC(frame,frameId);

    header.frameId = frameId;
    header.msgVersion = 3;

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
    pose.pose.pos.arr[1] = 2;
    pose.pose.pos.arr[2] = 2;

    /* Orientation */ 

    pose.pose.orient.nCount = 4;
    pose.pose.orient.arr[0] = 0;
    pose.pose.orient.arr[1] = 0;
    pose.pose.orient.arr[2] = 0;
    pose.pose.orient.arr[3] = 1;

}

void create_plan_point(int quarter,int xRange, int yRange, int zRange, double scale,double distanceFromRobot, PointCloudPoseStamped_pointCloudData &pc)
{
    int maxNPoints = 300000;
    int xInc = xRange/scale;
    int yInc = yRange/scale;
    int nbPointsPerCell = maxNPoints/(xInc*yInc);
    double xMult,yMult;
    double x_min = 0,x_max = 0, y_min = 0,y_max = 0,z_min=0,z_max=0,zSlope;
    double y;
    double x;
    double z;
    int cellIndex = 0;

    z_min -= zRange /2.0;
    z_max += zRange /2.0;

    cout << "Parameters: " << endl;
    cout << "z_min = " << z_min << endl;
    cout << "z_max = " << z_max << endl;
    cout << "xInc = " << xInc << endl;
    cout << "yInc = " << yInc << endl;
    cout << "nbPointsPerCell = " << nbPointsPerCell << endl;

    if (quarter > 3)
    {
        std::cerr << "Cannot create point cloud, give a valid quarter arg" << std::endl;
        return;
    }

    if (quarter%2 == 0)
    {
        zSlope = scale * (double)zRange/yRange;
        x_min -= xRange/2.0;
        x_max += xRange/2.0;
        cout << "x_min = " << x_min << endl;
        cout << "x_max = " << x_max << endl;
        cout << "zSlope = " << zSlope << endl;

        if (quarter == 0)
        {
            y_min = distanceFromRobot;
            y_max = y_min + yRange;
            yMult = 1.0;
        }
        else
        {
            y_min = -distanceFromRobot;
            y_max = y_min - yRange;
            yMult = -1.0;
        }

        cout << "y_min = " << y_min << endl;
        cout << "y_max = " << y_max << endl;
        cout << "yMult = " << yMult << endl;

        x = x_min;
        y = distanceFromRobot;
        z = z_min;

        for (int i = 0; i < yInc; i++)
        {
            x = x_min;
            for (int j = 0; j < xInc; j++)
            {
                for (int k = 0; k < nbPointsPerCell; k++,cellIndex++)
                {
                    if (i == 50 && j == 50 && k == 1)
                    {
                        cout << "[x,y,z] = " << x << ", " << y << ", " << z << endl;
                    }
                    pc.arr[cellIndex].arr[0] = x;
                    pc.arr[cellIndex].arr[1] = y;
                    pc.arr[cellIndex].arr[2] = z;
                    pc.arr[cellIndex].nCount = 3;
                }
                x += scale; 
            }
            y += scale*yMult;
            z += zSlope;
        }
    }
    else
    {
        zSlope = scale* (double)zRange/xRange;
        y_min -= yRange/2.0;
        y_max += yRange/2.0;
        if (quarter == 1)
        {
            x_min = distanceFromRobot;
            x_max = x_min + xRange;
            xMult = 1.0;
        }
        else
        {
            x_min = -distanceFromRobot;
            x_max = x_min - xRange;
            xMult = 1.0;
        }
        for (int i = 0; i < xInc; i++)
        {
            y = y_min;
            for (int j = 0; j < yInc; j++)
            {
                for (int k = 0; k < nbPointsPerCell; k++,cellIndex++)
                {
                    pc.arr[cellIndex].arr[0] = x;
                    pc.arr[cellIndex].arr[1] = y;
                    pc.arr[cellIndex].arr[2] = z;
                    pc.arr[cellIndex].nCount = 3;
                }
                y += scale;
            }
            x += scale*xMult; 
            z += zSlope;
        }
    }
    cout << "cellIndex = " << cellIndex << endl;
    pc.nCount = cellIndex; 
}
