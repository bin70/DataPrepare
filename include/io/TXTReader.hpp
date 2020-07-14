#pragma once
#include <common.hpp>

int frameid(std::string path)
{
    int begin = path.rfind('/');
    int end = path.rfind('.');
    std::string id_str = path.substr(begin + 1, end - begin - 1);
    return atoi(id_str.c_str());
}

class TXTReader
{
public:
    TXTReader(std::string txt_dir, int _data_columns = 4)
    {
        data_columns = _data_columns;
        openTXTDir(txt_dir);
    }

    bool readPointCloud(PointCloud::Ptr &cloud, int frame_id)
    {
        if (txt_files.count(frame_id) > 0)
            return readTXT(txt_files[frame_id], cloud);
        else
            return false;
    };

private:
    int data_columns;
    std::map<int, std::string> txt_files;
    inline float norm(PointType &p){ return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);}
    
    void openTXTDir(std::string path)
    {
        std::cout << "Opening directory:" << path << std::endl;
        assert(CheckFileExist(path.c_str()) == true);
        
        DIR *dir = opendir(path.c_str());
        dirent *ptr;
        while ((ptr = readdir(dir)) != NULL)
        {
            if (ptr->d_name[0] == '.') continue;
            
            std::string filepath = path + "/" + std::string(ptr->d_name); 
            int frame_id = frameid(filepath);

            // 文件名添加到列表
            txt_files[frame_id] = filepath;
        }
        closedir(dir);
    };

    bool readTXT(std::string file, PointCloud::Ptr &cloud)
    {
        std::ifstream fs(file.c_str());
        if (!fs.is_open() || fs.fail())
        {
            PCL_ERROR("Could not open: %s", file.c_str());
            return false;
        }
        std::string line;
        int line_cnts = 0;
        std::vector<std::string> st;
        cloud->clear();
        while (!fs.eof())
        {
            getline(fs, line);

            // Ignore lines
            if (line.empty())
                continue;

            // Tokenize the line
            boost::trim(line);
            boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);

            // x y z intensity
            if (st.size() != data_columns) continue;

            PointType p;
            p.x = val<float>(st[0]);
            p.y = val<float>(st[1]);
            p.z = val<float>(st[2]);
            
            // 25m
            if (norm(p) > 25.0) continue;
            
            p.intensity = val<float>(st[3]);
            p.curvature = val<float>(st[4]);
            cloud->points.push_back(p);
        }
        fs.close();
        cloud->width = uint32_t(cloud->points.size());
        cloud->height = 1;
        cloud->is_dense = true;

        return true;
    }
};
