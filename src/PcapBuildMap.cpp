#include <pcl/common/transforms.h>
#include <io/PcapReader.hpp>
#include <io/TrajIO.hpp>
#include <io/LasOperator.hpp>
#include <visualization/ShowCloud.hpp>
#include <build_map/MapManager.hpp>

#define USE_OCTO_MAP 1
#define USE_LOAM_POSE 0 // 是否使用LOAM的位姿表示方法(欧拉角)
#define TEST_TF_2_EULER 0

#if USE_LOAM_POSE
#include <loam/transform.hpp>
Twist _transformSum;
#endif

int main(int argc, char **argv){
    ArgParser args;
    if (args.parse_arg(argc, argv) == -1) 
      exit(-1);
    CreateDir(args.outputFolder.c_str());
    DeleteDir(args.temp_dir.c_str());

    pcl::visualization::PCLVisualizer* viewer;
    if(args.isShow)
        viewer = new pcl::visualization::PCLVisualizer("default");

    PointCloudReader reader;
    reader.setPcapFile(args.pcapFile);
    reader.setCalibFile(args.calibFile);
    reader.setVoxelSize(0.03); // 单帧分辨率
    reader.init(args);

    TrajIO traj(args.trajFile);

    // octree 的网格决定了地图的分辨率, 默认3(单位m)
    Octree::Ptr map_octree(new Octree((args.resolution)/100.0)); 
    PointCloud::Ptr map_cloud(new PointCloud);
    MapManager map(map_cloud, map_octree);
    
    PointCloud::Ptr cloud(new PointCloud);
    long long frameID = args.startID;
    consoleProgress(0);

    while( reader.readPointCloud(cloud, frameID) )
    {

        
        #if USE_LOAM_POSE // 用loam的坐标变换
            
            #if TEST_TF_2_EULER 
            // 测试通过
            // 说明变换矩阵表示的pose可以正确转回loam的欧拉角表示
            PoseNode p = traj.getPoseByFrameID(frameID);
            Twist _transformSum = qvToEuler(p.qua, p.pos);
            transformFullResToMap(cloud, _transformSum);
            #else
            Eigen::Matrix<double, 6, 1> loamTrans = traj.getPoseByFrameID(frameID).loamTrans;
            _transformSum.rot_x = Angle(loamTrans[0]);
            _transformSum.rot_y = Angle(loamTrans[1]);
            _transformSum.rot_z = Angle(loamTrans[2]);
            _transformSum.pos.x() = loamTrans[3];
            _transformSum.pos.y() = loamTrans[4];
            _transformSum.pos.z() = loamTrans[5];
            transformFullResToMap(cloud, _transformSum);
            #endif
        
            #else // 变换矩阵表示的位姿
            Eigen::Matrix4d m = traj.getPoseMatrix(frameID);
            pcl::transformPointCloud(*cloud, *cloud, m);
        #endif
        

        #if USE_OCTO_MAP
        // 添加到网格地图中
        map.AddFrameToMap(cloud);
        #else
        // 全分辨率
        *map_cloud += *cloud;
        #endif
        
        if(args.isShow)
            vis_utils::ShowCloud(map_cloud, viewer);

        // 间隔几帧
        frameID += args.frameGap; 
        
        // 结束帧
        if(frameID > args.endID) 
            break;

        consoleProgress(frameID, args);
    }

    saveLasFile(args.outputMap, map_cloud);
    std::cout << "地图保存至: " << args.outputMap << std::endl;
    
    return 0;
}
