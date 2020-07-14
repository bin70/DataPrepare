#include <pcl/common/transforms.h>
#include <io/PcapReader.hpp>
#include <io/TrajIO.hpp>
#include <io/LasOperator.hpp>
#include <visualization/ShowCloud.hpp>
#include <build_map/MapManager.hpp>
#include <argparse.hpp>

#define USE_OCTO_MAP 1
#define USE_LOAM_POSE 0 // 是否使用LOAM的位姿表示方法(欧拉角)
#define TEST_TF_2_EULER 0

#if USE_LOAM_POSE
#include <loam/transform.hpp>
Twist _transformSum;
#endif

std::string getCalibFile(int data_type)
{
    switch (data_type)
    {
    case 0:
        return "../resource/VLP-16.xml";
    case 1:
        return "../resource/HDL-32.xml";
    case 2:
    //    return "../resource/VLP-32a.xml";
    //case 3:
        return "../resource/VLP-32c.xml";
    //case 4:
    //    return "../resource/VLP-32C.xml";
    default:
        std::cout << "Data type error!" << std::endl;
    }
}

Eigen::Matrix4d loadCalibMatrix()
{
    std::ifstream matrix_file("../resource/autoCalibMatrix.txt");
    Eigen::Matrix4d matrix;
    for(int i=0; i<4; ++i)
        for(int j=0; j<4; ++j)
            matrix_file >> matrix(i,j);
    return matrix;
}

pcl::visualization::PCLVisualizer *viewer;
bool is_show = false;
float resolution = 0.03;
int begin_id = 0;
int end_id = -1;
std::string out_dir = "./result";

int main(int argc, const char **argv)
{
    ArgumentParser parser;
    parser.addArgument("-p", "--pcap", true);
    parser.addArgument("-t", "--traj", true);
    parser.addArgument("-d", "--data_type", true);
    parser.addArgument("-b", "--begin_id");
    parser.addArgument("-e", "--end_id");
    parser.addArgument("--pcap2");
    parser.addArgument("-o", "--out_dir");
    parser.addArgument("-s", "--show");
    parser.addArgument("-r", "--resolution");
    parser.parse(argc, argv);

    if(parser.count("out_dir"))
        out_dir = parser.get("out_dir");
    
    CreateDir(out_dir.c_str());

    if (parser.count("show"))
    {   
        is_show = parser.get<bool>("show");
        viewer = new pcl::visualization::PCLVisualizer("default");
    }

    if(parser.count("begin_id"))
        begin_id = parser.get<int>("begin_id");
    
    if(parser.count("end_id"))
        end_id = parser.get<int>("end_id");

    PointCloudReader reader;
    reader.setPcapFile(parser.get("pcap"));
    reader.setCalibFile(getCalibFile(parser.get<int>("data_type")));
    reader.setVoxelSize(0.03); // 单帧分辨率
    reader.setValidDistance(25.0);
    reader.init();

    TrajIO traj(parser.get("traj"), G2O, 3);

    MapManager map(resolution);
    map.update();

    PointCloud::Ptr cloud(new PointCloud);
    long long frameID = begin_id;
    consoleProgress(0);

    // 把201标定到202上的矩阵
    Eigen::Matrix4d calibMatrix = loadCalibMatrix();

    while (reader.readPointCloud(cloud, frameID))
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
        //m = m*calibMatrix;
        pcl::transformPointCloud(*cloud, *cloud, m);
#endif

#if USE_OCTO_MAP
        // 添加到网格地图中
        map.AddFrameToMap(cloud);
#else
        // 全分辨率
        *map_cloud += *cloud;
#endif
        if (is_show)
        #if 1
            vis_utils::ShowCloud(map.getMapPtr(), viewer);
        #else
        {
            vis_utils::ShowCloud(cloud, viewer, "intensity", 3);
            vis_utils::waitForSpace(viewer);
        }
        #endif
        // 间隔几帧
        frameID += traj.getFrameGap();

        // 结束帧
        if (frameID > end_id)
            break;

        consoleProgress(frameID, begin_id, end_id);
    }

    std::stringstream out;
    out << out_dir << "/map_" << begin_id << "_" << end_id << ".las"; 
    
    saveLasFile(out.str(), map.getMapPtr());
    std::cout << "地图保存至: " << out.str() << std::endl;

    return 0;
}
