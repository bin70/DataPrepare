#include <pcl/common/transforms.h>
#include <io/TXTReader.hpp>
#include <common.hpp>
#include <io/TrajIO.hpp>
#include <io/LasOperator.hpp>
#include <build_map/MapManager.hpp>
#include <visualization/ShowCloud.hpp>
#include <argparse.hpp>

#define USE_OCTO_MAP 1
#define USE_LOAM_POSE 0 // 是否使用LOAM的位姿表示方法(欧拉角)
#define TEST_TF_2_EULER 0

#if USE_LOAM_POSE
#include <loam/transform.hpp>
Twist _transformSum;
#endif

/***********************************
 * 说明：
 * 本程序是将从KITTI bin转换来的txt格式的扫描帧，
 * 通过KITTI格式的位姿数据合并成地图的程序
 **********************************/

using namespace std;
using namespace vis_utils;
using namespace pcl::visualization;

PCLVisualizer *viewer;
bool show_cloud = false;
float resolution = 0.03;


void saveCloud(string out_path, PointCloud::Ptr &cloud)
{
  if(cloud->points.size()==0)
  {
      std::cout << out_path << " is empty!" << std::endl;
      return;
  }
  cloud->height = 1;
  cloud->width = cloud->points.size();
  cloud->is_dense = true;
  pcl::io::savePCDFileASCII(out_path, *cloud);
}

int main(int argc, const char **argv){
    ArgumentParser parser;
    parser.addArgument("-i", "--input_dir", true);
    parser.addArgument("-b", "--begin_id", true);
    parser.addArgument("-e", "--end_id", true);
    parser.addArgument("-r", "--resolution");
    parser.addArgument("-s", "--show_cloud");
    parser.parse(argc, argv);

    consoleProgress(0);

    if(parser.count("resolution"))
        resolution = parser.get<float>("resolution");
    
    if(parser.count("show_cloud"))
    {
        show_cloud = parser.get<bool>("show_cloud");
        viewer = new PCLVisualizer("Labeled Scan");
        viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);
    }

    string input_dir = parser.get("input_dir");
    int begin_id = parser.get<int>("begin_id");
    int end_id = parser.get<int>("end_id");
    string out_dir = input_dir+"/merged_"+to_string(begin_id)+"_"+to_string(end_id);
    createDir(out_dir);

    TXTReader reader(input_dir+"/labeled_scans", 5); // xyzil
    TrajIOKITTI traj(input_dir+"/poses.txt", begin_id, end_id);
    std::cout << "Read trajectory finished." << std::endl;

    // octree 的网格决定了地图的分辨率, 默认3cm
    MapManager map(resolution);
    map.update();
    
    PointCloud::Ptr cloud(new PointCloud);
    
    std::cout << "Start to merging labeled scans..." << std::endl;
    int frame_id = begin_id;
    while(reader.readPointCloud(cloud, frame_id))
    {
       
        Eigen::Matrix4d m = traj.getPose(frame_id);
        pcl::transformPointCloud(*cloud, *cloud, m);

        #if USE_OCTO_MAP
        // 添加到网格地图中
        map.AddFrameToMap(cloud);
        #else
        // 全分辨率
        *map_cloud += *cloud;
        #endif
        if(show_cloud)
            vis_utils::ShowCloud(map.getMapPtr(), viewer);

        // 间隔几帧
        frame_id ++;
        
        // 结束帧
        if(frame_id > end_id) 
            break;

        consoleProgress(frame_id, begin_id, end_id);
    }

    std::cout << "Saving map into 4 classes..." << std::endl;
    PointCloud::Ptr map_cloud = map.getMapPtr();
    PointCloud::Ptr floor(new PointCloud);
    PointCloud::Ptr ceiling(new PointCloud);
    PointCloud::Ptr wall(new PointCloud);
    PointCloud::Ptr clutter(new PointCloud);
    PointCloud::Ptr unused(new PointCloud);

    for(int i=0; i<map_cloud->points.size(); ++i)
    {
        PointType p = map_cloud->points[i];
        switch ((int)p.curvature)
        {
        case 0:
            unused->points.push_back(p);
            break;
        case 1:
            floor->points.push_back(p);
            break;
        case 2:
            ceiling->points.push_back(p);
            break;
        case 3:
            wall->points.push_back(p);
            break;
        default:
            clutter->points.push_back(p);
            break;
        }
    }

    saveLasFile(out_dir+"/merged_map_"+to_string(begin_id)+"_"+to_string(end_id)+".las", map_cloud);
    saveCloud(out_dir+"/unused_1.txt", unused);
    saveCloud(out_dir+"/floor_1.txt", floor);
    saveCloud(out_dir+"/ceiling_1.txt", ceiling);
    saveCloud(out_dir+"/wall_1.txt", wall);
    saveCloud(out_dir+"/clutter_1.txt", clutter);
    std::cout << "地图保存至: " << out_dir << std::endl;
    return 0;
}
