#pragma once

#include <common.hpp>
#include <map>
#include <pcl/io/pcd_io.h>

enum LABEL
{
    FLOOR,    // 地面0
    WALL,     // 墙面1
    CEILING,  // 顶面2
    UNLABELED // 其他3
};

int labelid(std::string path)
{
    int begin = path.rfind('/');
    int end = path.rfind('.');
    std::string id_str = path.substr(begin + 1, end - begin - 1);
    return atoi(id_str.c_str());
}

class SemanticMap
{
public:
    SemanticMap() {}

    SemanticMap(PointCloud::Ptr &map_cloud, std::string map_folder)
    {
        std::cout << "Loading semantic map ..." << std::endl;
        loadMap(map_cloud, map_folder);
    }

    bool loadMap(PointCloud::Ptr &map_cloud, std::string rootDir)
    {
        if (!openDir(rootDir))
            return false;

        for (LABEL label = FLOOR; label <= UNLABELED; label = (LABEL)(label + 1))
        {
            std::ifstream fcloud(file_list[label]);
            if (!fcloud.is_open())
            {
                std::cout << "打开文件" << file_list[label] << "失败" << std::endl;
                return false;
            }

            PointType p;
            while (!fcloud.eof())
            {
                fcloud >> p.x;

                if (fcloud.eof())
                    break;

                fcloud >> p.y >> p.z >> p.intensity;
                p.curvature = (float)label;
                map_cloud->points.push_back(p);
            }
        }
        return true;
    }

    bool openDir(std::string rootDir)
    {
        if (access(rootDir.c_str(), 0) != 0)
            return false;

        DIR *dir = opendir(rootDir.c_str());
        dirent *ptr;
        while (ptr = readdir(dir))
        {
            if (ptr->d_name[0] == '.')
                continue;

            std::string filePath = rootDir + "/" + std::string(ptr->d_name);
            int label = labelid(filePath);
            file_list[label] = filePath;
        }
        return true;
    }

private:
    std::map<int, std::string> file_list;
};