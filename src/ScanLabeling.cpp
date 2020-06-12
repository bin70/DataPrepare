#include <common.hpp>
#include <pcl/common/transforms.h>
#include <io/PCDReader.hpp>
#include <io/TrajIO.hpp>
#include <build_map/MapManager.hpp>
#include <visualization/ShowCloud.hpp>
#include <argparse.hpp>
#include <semantic_map/SemanticMap.hpp>

using namespace std;
using namespace vis_utils;
using namespace pcl::visualization;

PCLVisualizer *viewer;
bool show_cloud = false;
float resolution = 0.03;

int main(int argc, const char **argv)
{
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
    string out_dir = input_dir+"/labeled_scans";
    createDir(out_dir);

    PCDReader reader(input_dir+"/raw_scans");
    TrajIO traj(input_dir+"/traj_with_timestamp.txt");
    MapManager map(resolution);
    SemanticMap smap(map.getMapPtr(), input_dir+"/labeled_map");
    map.update();

    int begin_id = parser.get<int>("begin_id");
    int end_id = parser.get<int>("end_id");
    
    PointCloud::Ptr cloud(new PointCloud);
    PointCloud::Ptr cloud_filtered(new PointCloud);
    PointType pointSel;
    pcl::VoxelGrid<PointType> grid;
    grid.setLeafSize(resolution, resolution, resolution);

    int frame_id = begin_id;
    while(reader.readPointCloud(cloud, frame_id))
    {
        grid.setInputCloud(cloud);
        grid.filter(*cloud_filtered);

        Eigen::Matrix4d m = traj.getPoseMatrix(frame_id);
        pcl::transformPointCloud(*cloud_filtered, *cloud, m);

        for(auto &point : *cloud)
        {
            if(map.getNearestPoint(point, pointSel))
                point.curvature = pointSel.curvature;
            else
                point.curvature = 0;
        }

        if(show_cloud)
        {
            ShowCloud(cloud, viewer, "curvature");
            waitForSpace(viewer);
        }
        
        
        pcl::io::savePCDFile(out_dir+"/"+to_string(frame_id)+".pcd", *cloud);

        frame_id += traj.getFrameGap();

        consoleProgress(frame_id, begin_id, end_id);
    }

    return 0;
}