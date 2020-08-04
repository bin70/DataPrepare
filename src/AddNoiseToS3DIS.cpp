#include <point_cloud/common.h>
#include <pcl/io/ply_io.h>
#include <boost/random.hpp>
#include <pcl/console/time.h>

#include <io/FileOperator.hpp>
#include <argparse.hpp>

using namespace std;

ArgumentParser parser;
FileOperator fop;

void addNoise(pcl::PointCloud<pcl::PointXYZI> &cloud, float mu, float sigma)
{
    boost::mt19937 rng;
    rng.seed(static_cast<unsigned int>(time(0)));
    boost::normal_distribution<> nd(mu, sigma);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> var_nor(rng, nd);

    //添加噪声
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        cloud.points[i].x += static_cast<float> (var_nor());
        cloud.points[i].y += static_cast<float> (var_nor());
        cloud.points[i].z += static_cast<float> (var_nor());
    }
}

int main(int argc, const char **argv)
{
    parser.addArgument("-i", "--input", true);
    parser.addArgument("-m", "--mu", true);
    parser.addArgument("-s", "--sigma", true);
    parser.addArgument("-o", "--output");
    parser.parse(argc, argv);

    string input = parser.get("input");
    string output = fop.getParentDir(input) + "/noised";
    if(parser.count("output"))
        output = parser.get("output");
    
    vector<string> filelist = fop.openDir(parser.get("input"));
    pcl::PLYReader reader;
    pcl::PLYWriter writer;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    fop.makeDir(output);
    for(int i=0; i<filelist.size(); ++i)
    {
        reader.read<pcl::PointXYZI>(filelist[i], cloud);
        cout << "prossing " << filelist[i] << endl;
        addNoise(cloud, parser.get<float>("mu"), parser.get<float>("sigma"));
        
        // ascii格式的RandLA-Net的脚本无法处理
        // writer.write<pcl::PointXYZI>(output+"/"+fop.getFileName(filelist[i])+".ply", cloud);
        
        // 存为二进制
        pcl::PCLPointCloud2 cloud2;
        pcl::toPCLPointCloud2(cloud, cloud2);
        writer.writeBinary(output+"/"+fop.getFileName(filelist[i])+".ply", cloud2, 
            Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false); // 不使用相机
    }
    
    return 0;
}
