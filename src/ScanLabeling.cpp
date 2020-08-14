// pcl
#include <point_cloud/common.hpp>

// Toolkit lib
#include <utils/argparse.hpp>
#include <utils/common.hpp>
#include <io/TrajIO.hpp>
#include <io/PCDOperator.hpp>
#include <build_map/MapManager.hpp>
#include <visualization/ShowUtils.hpp>

// own
#include <SemanticMap.hpp>

using namespace std;
using namespace pcl::visualization;

ShowUtils su;
bool ShowUtils::isPause = true;

FileOperator fop;
bool show_cloud = false;
float resolution = 0.1; // 默认10cm网格

int main(int argc, const char **argv)
{
    ArgumentParser parser;
    parser.addArgument("-i", "--input_dir", true);
    parser.addArgument("-b", "--begin_id", true);
    parser.addArgument("-e", "--end_id", true);
    parser.addArgument("-t", "--traj_type", true);
    parser.addArgument("-o", "--out_dir", true);
    parser.addArgument("-r", "--resolution");
    parser.addArgument("-s", "--show_cloud");
    parser.parse(argc, argv);

    if(parser.count("resolution"))
        resolution = parser.get<float>("resolution");
    
    if(parser.count("show_cloud"))
    {
        show_cloud = parser.get<bool>("show_cloud");
        su.init("Labeled Scan", &ShowUtils::keyboardEvent);
    }

    string input_dir = parser.get("input_dir");
    string pcd_dir = input_dir+"/raw_scans";
    string traj_path = input_dir+"/traj_with_timestamp.txt";
    string labeled_map = input_dir+"/labeled_map";
    TrajType traj_type = (TrajType)parser.get<int>("traj_type");

    PCDReader reader(pcd_dir, true); // 使用的是二进制文件
    TrajIO traj(traj_path, traj_type);
    
    SemanticMap smap(labeled_map, resolution);

    if(show_cloud)
    {
        su.ShowCloud(smap.getMapPtr(), "map", "curvature", 2);
        su.waitSpace();
    }
    
    int begin_id = parser.get<int>("begin_id");
    int end_id = parser.get<int>("end_id");
    
    PointCloud::Ptr cloud(new PointCloud);
    PointCloud::Ptr cloud_filtered(new PointCloud);
    PointType pointSel;
    pcl::VoxelGrid<PointType> grid;
    grid.setLeafSize(resolution, resolution, resolution);

    std::cout << "Start to labeling scans..." << std::endl;
    consoleProgress(0);

    string out_dir = parser.get("out_dir");;
    fop.makeDir(out_dir);

    int frame_id = begin_id;
    while(reader.readPointCloud(cloud, frame_id))
    {
        grid.setInputCloud(cloud);
        grid.filter(*cloud_filtered);

        Eigen::Matrix4d m = traj.getPoseMatrix(frame_id);
        pcl::transformPointCloud(*cloud_filtered, *cloud, m);

        for(auto &point : *cloud)
        {
            if(smap.searchPoint(point, pointSel))
                point.curvature = pointSel.curvature;
            else
                point.curvature = CLUTTER;
        }

        Eigen::Matrix4d m_inv = m.inverse();
        pcl::transformPointCloud(*cloud, *cloud, m_inv);

        if(show_cloud)
        {
            su.ShowCloud(cloud, "frame", "curvature", 4);
            su.waitSpace();
        }

        pcl::io::savePCDFileBinaryCompressed(out_dir+"/"+to_string(frame_id)+".pcd", *cloud);

        frame_id += traj.getFrameGap();
        consoleProgress(frame_id, begin_id, end_id);
        if(frame_id > end_id) break;
    }

    std::cout << "Save result to " << out_dir << std::endl;

    return 0;
}