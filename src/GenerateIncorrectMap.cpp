// system
#include <random>
#include <ctime>

// pcl
#include <pcl/common/transforms.h>

// Toolkit lib
#include <io/PCDOperator.hpp>
#include <io/TrajIO.hpp>
#include <io/FileOperator.hpp>
#include <build_map/MapManager.hpp>
#include <visualization/ShowCloud.hpp>
#include <argparse.hpp>
#include <math/lie_algebra.h>
#include <math/common.h>
#include <math/TransformTool.hpp>

// own
#include <SemanticMap.hpp>

using namespace std;
using namespace Eigen;
using namespace vis_utils;
using namespace pcl::visualization;

typedef Matrix<double, 6, 1> Vector6d;
PCLVisualizer *viewer;
FileOperator fop;
TransformTool tt;
bool show_cloud = false;
float resolution = 0.03;

VectorXd caculateLinearIncre(Matrix4d initialNoise, int steps)
{
    VectorXd init = SE3::log(initialNoise);
    return init/steps;
}

// 产生一个微小扰动
Matrix4d generateNoise(int dim)
{
    Vector6d noise_euler;
    
    if(dim ==0 || dim == 1)
        noise_euler(dim) = d2r(-1.0);
    else
        noise_euler(dim) = d2r(0.5);

    // float angle_range = 10.0;
    // float distance_range = 0.15;
    // default_random_engine e(time(0));
    // uniform_real_distribution<double> angle_error(-angle_range, angle_range);
    // uniform_real_distribution<double> distance_error(-distance_range, distance_range);
    
    return tt.euler2matrix(noise_euler);
}

// 在流形上进行递减的线性扰动
Matrix4d generateNewNoise(Matrix4d old_noise, VectorXd inc)
{
    VectorXd new_noise = SE3::log(old_noise) - inc;
    return SE3::exp(new_noise);
}

void saveLabelCloud(string save_dir, PointCloud::Ptr cloud)
{
    ofstream ceiling_txt(save_dir+"/ceiling_1.txt");
    ofstream floor_txt(save_dir+"/floor_1.txt");
    ofstream wall_txt(save_dir+"/wall_1.txt");
    ofstream clutter_txt(save_dir+"/clutter_1.txt");
    
    for(int i=0; i<cloud->points.size(); ++i)
    {
        PointType p = cloud->points[i];
        if((int)p.curvature == label_map["ceiling"])
            ceiling_txt << p.x << " " << p.y << " " << p.z << " " <<  p.intensity << std::endl;
        else if ((int)p.curvature == label_map["floor"])
            floor_txt << p.x << " " << p.y << " " << p.z << " " <<  p.intensity << std::endl;
        else if ((int)p.curvature == label_map["wall"])
            wall_txt << p.x << " " << p.y << " " << p.z << " " <<  p.intensity << std::endl;
        else
            clutter_txt << p.x << " " << p.y << " " << p.z << " " <<  p.intensity << std::endl;
    }
    ceiling_txt.close();
    floor_txt.close();
    wall_txt.close();
    clutter_txt.close();
}

int frame_gap;
int map_size;
bool enable_noise = true;
default_random_engine e(time(0));

void generateRandom(int &error_begin, int &error_length)
{
    // 随机的出错长度
    uniform_int_distribution<unsigned> rand_error_length(20, 50);
    error_length = rand_error_length(e);
    
    // 随机的出错位置
    uniform_int_distribution<unsigned> rand_error_begin(0, map_size*frame_gap-error_length);
    
    error_begin = rand_error_begin(e);

    std::cout << "error length = " << error_length << std::endl;
    std::cout << "error begin = " << error_begin <<  std::endl;
}

int main(int argc, const char **argv)
{
    ArgumentParser parser;
    parser.addArgument("-i", "--input_dir", true); // 输入文件夹
    parser.addArgument("-m", "--map_size", true); // 地图的帧数，先取100帧
    parser.addArgument("-e", "--error_type", true); // 错误的种类, x|y|z轴的旋转和平移
    parser.addArgument("-t", "--traj_type", true);
    parser.addArgument("-o", "--out_dir");
    parser.addArgument("-r", "--resolution");
    parser.addArgument("-s", "--show_cloud");
    parser.addArgument("-n", "--noise_add");
    parser.parse(argc, argv);

    if(parser.count("resolution"))
        resolution = parser.get<float>("resolution");
    
    if(parser.count("show_cloud"))
    {
        show_cloud = parser.get<bool>("show_cloud");
        viewer = new PCLVisualizer("Labeled Local Map");
        viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);
    }

    string input_dir = parser.get("input_dir");
    string pcd_dir = input_dir+"/labeled_scans";
    string traj_path = input_dir+"/traj_with_timestamp.txt";
    TrajType traj_type = (TrajType)parser.get<int>("traj_type");

    PCDReader reader(pcd_dir, true); // 使用的是二进制文件
    TrajIO traj(traj_path, traj_type);
    
    MapManager map(resolution);
    // SemanticMap smap(map.getMapPtr(), labeled_map);
    map.update();

    int begin_id = traj.getStartID(); // parser.get<int>("begin_id");
    int end_id = traj.getEndID(); // parser.get<int>("end_id");
    frame_gap = traj.getFrameGap();
    map_size = parser.get<int>("map_size");

    PointCloud::Ptr cloud(new PointCloud);

    string out_dir = input_dir + "/incorrect_map";
    if(parser.count("out_dir"))
        out_dir = parser.get("out_dir");
    fop.makeDir(out_dir);

    Matrix4d noise;
    string error_name;

    if(parser.get<int>("error_type") == 0)
    {
        error_name = "X_rot";
        // z轴平移, 并发生一定x轴的旋转
        noise = generateNoise(0);
    }
    else if(parser.get<int>("error_type") == 1)
    {
        error_name = "Y_rot";
        noise = generateNoise(1);
    }
    else if(parser.get<int>("error_type") == 2)
    {
        error_name = "Z_rot";
         // z轴旋转大概5度
        noise = generateNoise(2);
    }
    else if(parser.get<int>("error_type") == 3)
    {
        error_name = "Initial";
        enable_noise = false;
    }
        
    int frame_id = begin_id;
    
    consoleProgress(0);
   
    int frame_count = 0;
    int map_start_id = frame_id;
    int error_begin, error_length;
    
    generateRandom(error_begin, error_length);
    // 计算线性干扰量
    VectorXd noiseIncre = caculateLinearIncre(noise, error_length);
    Matrix4d noiseRecover = noise;

    while(reader.readPointCloud(cloud, frame_id))
    {

        Eigen::Matrix4d m = traj.getPoseMatrix(frame_id);
        if(enable_noise && frame_id >= map_start_id + error_begin
            && frame_id <= map_start_id + error_begin + error_length*frame_gap)
        {
            m = noise * m;
            noise = generateNewNoise(noise, noiseIncre);
        }
        
        pcl::transformPointCloud(*cloud, *cloud, m);

        map.addFrame(cloud);
        frame_count++;

        if(show_cloud)
            ShowCloud(map.getMapPtr(), viewer, "curvature", 2);

        consoleProgress(frame_id, begin_id, end_id);
        
        if(frame_count % map_size == 0)
        {
            string map_out_dir = out_dir+"/"+ error_name // 对应区域文件夹
                +"/map_"+to_string(map_start_id)+"_"+to_string(frame_id) // 对应房间文件夹
                +"/Annotations";
            fop.makeDir(map_out_dir);
            std::cout << "Saving map generated from frame " << map_start_id << " to " << frame_id << "..." << std::endl;
            saveLabelCloud(map_out_dir, map.getMapPtr());
            pLine();

            map.UpdateMap(cloud);

            // reset some state
            noise = noiseRecover;
            if(frame_id == 648)
            {
                error_begin = 30;
                error_length = 40;    
            }
            else
            {
                generateRandom(error_begin, error_length);
            }

            noiseIncre = caculateLinearIncre(noise, error_length);
            map_start_id = frame_id + frame_gap;
        }

        frame_id += frame_gap;
        if(frame_id > end_id) break;
    }

    std::cout << "Result saved to " << out_dir << std::endl;
    return 0;
}