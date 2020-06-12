#pragma once

#ifndef COMMON_H
#define COMMON_H
//----------system operation----------
#define linux 1
#ifdef linux
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#endif
#ifdef WIN32
#include <io.h>
#include <direct.h>
#endif

//-----------std headers-------------------------
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cmath>
#include <stdlib.h>
#include <string.h>
//------------pcl headers------------------------
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// eigen
#include <Eigen/Dense>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloud;

inline std::string getFileName(std::string path)
{
    std::string fullName = path.substr(path.rfind('/') + 1);
    return fullName.substr(0, fullName.find('.'));
}

///************************************************************************/
///* 创建文件夹，存在则不创建                                                  */
///************************************************************************/
inline int CreateDir(const char *sPathName)
{
    char DirName[256];
    strcpy(DirName, sPathName);
    int i, len = strlen(DirName);
    if (DirName[len - 1] != '/')
        strcat(DirName, "/");
    len = strlen(DirName);
    for (i = 1; i < len; i++)
    {
        if (DirName[i] == '/' || DirName[i] == '\\')
        {
            DirName[i] = 0;
            if (access(DirName, 0) != 0) //存在则返回0
            {
                if (mkdir(DirName, 0755) == -1)
                {
                    perror("mkdir   error");
                    return -1;
                }
            }
            DirName[i] = '/';
        }
    }

    return 0;
}

inline int createDir(std::string path)
{
    return CreateDir(path.c_str());
}

///************************************************************************/
///* 删除文件夹，不存在则不删除                                               */
///************************************************************************/
inline int DeleteDir(const char *path)
{
    std::string dirPath = path;
    if (access(path, 0) == 0)
    {
        std::string shell_delete_command = "rm -rf " + dirPath;
        return system(shell_delete_command.c_str());
    }
    return 0;
}

///************************************************************************/
///* 检查文件是否存在
///************************************************************************/

inline bool CheckFileExist(const char *p)
{

    if ((access(p, 0)) == 0)
    {
        return true;
    }
    return false;
}

inline void consoleProgress(int progress)
{
    static char bar[102] = {0};
    static std::string lable = "|/-\\";
    static int inprogress = -1;
    if (inprogress == progress)
        return;

    int st = progress / 2;
    for (int i = 0; i < st; i++)
    {
        bar[i] = '#';
    }
    bar[st] = 0;

    printf("[%-50s][%d%%][%c]\r", bar, progress, lable[progress % 4]);
    fflush(stdout);

    inprogress = progress;

    if (progress == 100)
    {
        printf("\n");
    }
}

inline void consoleProgress(int progress, int begin, int end)
{
    consoleProgress(100. * (progress - begin) / (end - begin));
}

template <typename T>
T val(const std::string &s)
{
    std::stringstream ss;
    ss << s;
    T v;
    ss >> v;
    return v;
}

class ArgParser
{
public:
    ArgParser() { init(); }

    ArgParser(int n_args, char **argList)
    {
        init();
        if (parse_arg(n_args, argList) == -1)
            printHelp();
    }

    ~ArgParser() {}

    void init()
    {
        pcapFile = "";
        calibFile = "./resource/HDL-32.xml";
        pcap2File = "";
        calibMatrix = "./resource/autoCalibMatrix.txt"; //外标定文件
        outputFolder = "./output";
        trajFile = "";
        data_flag = 1; //默认32线
        valid_dis = 40.0;
        nscans = 32;
        startID = 0;
        endID = -1;
        resolution = 3; //cm
        frameGap = 2;
        isMerg2Cloud = false;
        isShow = false;
        isShowCloud = false; // 最后建图的可视化
        arg_flag = false;    // 检查参数是否合法(true表示出现不合法参数)
        processFlag = 0;
        traj_type = 0; //旧轨迹
    }

    void printHelp()
    {
        printf("-f\t设置PCAP文件路径！\n");
        printf("-m\t设置第二个PCAP文件路径！\n");
        printf("-d\t设置为16线还是32线的数据。如不设置默认为32线（0：16线 1：32线）！\n");
        printf("-b\t设置建图起始帧数。默认从第一帧开始!\n");
        printf("-e\t设置建图结束的帧数。默认建图到最后一帧!\n");
        printf("-g\t设置建图频数。默认每2帧进行一次mapping!\n");
        printf("-r\t设置建图密度。默认3cm(输入整数)!\n");
        printf("-s\t设置前端建图可视化(0:不显示 1:显示轨迹 2:显示轨迹和点云)!\n");
        printf("-o\t设置输出文件路径。如不设置默认为可执行文件路径下的output文件夹！\n");
        printf("-v\t设置有效的雷达数据范围,默认40米\n");
        printf("-p\t设置处理点云帧的程序块:\n\t1.生成网络训练数据 \n\t2.建图和可视化 \n\t3.loam \n\t4.gicp \n\t5.fusion\n");
        printf("-t\t设置轨迹文件路径.\n");
        printf("-a\t设置轨迹类型. (0原始轨迹 1带位姿标签)\n");
        exit(0);
    }

    int parse_arg(int n_args, char **argList)
    {
        argc = n_args;
        argv = argList;
        for (i = 1; i < n_args; i++) // i是指向当前命令行参数的游标
        {
            currArg = argv[i];
            if (match("--help"))
                printHelp();

            if (check_arg("-f"))
                pcapFile = argv[i];
            else if (check_arg("-m"))
                pcap2File = argv[i];
            else if (check_arg("-d"))
                data_flag = std::atoi(argv[i]);
            else if (check_arg("-b"))
                startID = std::atoi(argv[i]);
            else if (check_arg("-e"))
                endID = std::atoi(argv[i]);
            else if (check_arg("-o"))
                outputFolder = argv[i];
            else if (check_arg("-g"))
                frameGap = std::atoi(argv[i]);
            else if (check_arg("-r"))
                resolution = std::atoi(argv[i]);
            else if (check_arg("-v"))
                valid_dis = atof(argv[i]);
            else if (check_arg("-p"))
                processFlag = atoi(argv[i]);
            else if (check_arg("-t"))
                trajFile = argv[i];
            else if (check_arg("-a"))
                traj_type = atoi(argv[i]);
            else if (check_arg("-s"))
            {
                isShowTraj = match("0") ? 0 : 1;
                isShowCloud = match("2") ? 1 : 0;
            }
            else
            {
                std::cout << "\n参数设置错误" << std::endl;
                printHelp();
            }
        }

        // 检查参数
        if (!(file_utils::CheckFileExist(pcapFile.c_str())))
        {
            printf("PCAP文件: %s 不存在!\n", pcapFile.c_str());
            return -1;
        }
        if (pcap2File.size() > 0)
        {
            if (!(file_utils::CheckFileExist(pcap2File.c_str())))
            {
                printf("PCAP2文件: %s 不存在!\n", pcap2File.c_str());
                exit(-1);
            }
            else
                isMerg2Cloud = true;
        }

        if (startID < 0)
        {
            printf("设置起始帧数错误！(必须大于等于0)");
            return -1;
        }
        if (endID != -1 && (endID <= startID || endID < 1))
        {
            printf("设置起始帧数错误！(必须大于起始帧数)");
            return -1;
        }
        if (frameGap < 1 || frameGap > 3)
        {
            printf("设置建图频数错误！(须在1-3之间)");
            return -1;
        }
        if (resolution <= 0 || resolution > 10)
        {
            printf("设置建图密度错误！(范围在1-10cm)");
            return -1;
        }

        if (data_flag < 0 || data_flag > 5)
        {
            printf("设置数据类型错误!(仅支持0,1,2, 3, 4, 5)");
            return -1;
        }

        switch (data_flag)
        {
        case 0:
            nscans = 16;
            calibFile = "./resource/VLP-16.xml";
            break;
        case 1:
            nscans = 32;
            calibFile = "./resource/HDL-32.xml";
            break;
        case 2:
            nscans = 32;
            calibFile = "./resource/VLP-32a.xml";
            break;
        case 3:
            nscans = 32;
            calibFile = "./resource/VLP-32b.xml";
            break;
        case 4:
            nscans = 32;
            calibFile = "./resource/VLP-32C.xml";
            break;
        };

        // 输出目录
        rootOutputDir = outputFolder; //输出的根目录,默认./output
        outputFolder = outputFolder + "/" + file_utils::getFileName(pcapFile);
        if (trajFile == "")
            trajFile = outputFolder + "/traj_" + std::to_string(startID) + "_" + std::to_string(endID) + ".txt";
        outputMap = outputFolder + "/map_" + std::to_string(startID) + "_" + std::to_string(endID) + ".las";

        // 临时文件
        temp_dir = rootOutputDir + "/tmp";
        tempTraj = temp_dir + "/temp_traj";

        // 和python通信用的文件
        // rootOutputDir的值由用户指定,因此不应该写在构造函数中
        label_0 = temp_dir + "/label_0";
        label_1 = temp_dir + "/label_1";
        temp_writing_flag = temp_dir + "/c_is_writing";

        return 0;
    }

    void print()
    {
        std::cout << "\n\n=============================== parameter ===================================\n"
                  << "pcap File=" << pcapFile << std::endl;

        if (isMerg2Cloud)
            std::cout << "pcap2 File=" << pcap2File << std::endl;

        std::cout << "output Traj=" << trajFile << std::endl
                  << "output Map=" << outputMap << std::endl
                  << "pcap1 Nscans=" << nscans << std::endl
                  << "==============================================================================" << std::endl;
    }

    int getDataType() { return data_flag; }

public:
    std::string pcapFile;
    std::string calibFile;

    std::string pcap2File;
    std::string calibMatrix;

    std::string rootOutputDir;
    std::string outputFolder;
    std::string outputMap;
    std::string trajFile;

    //用于存放中间数据
    std::string temp_dir;
    std::string tempTraj;

    // 用于和python通信的临时文件
    std::string label_0;
    std::string label_1;
    std::string temp_writing_flag;

    int processFlag;
    float valid_dis;
    int nscans;
    int startID;
    int endID;
    int resolution; //cm
    int frameGap;
    bool isMerg2Cloud;
    bool isShow;
    bool isShowCloud; //建图阶段是否可视化点云
    bool isShowTraj;  //仅仅可视化轨迹
    int traj_type;

private:
    int i;
    int argc;
    int check_f;
    const char *currArg;
    char **argv;
    bool arg_flag;
    int data_flag;
    inline bool match(const char *value)
    {
        return (strcmp(argv[i], value) == 0);
    }

    int check_arg(const char *flag)
    {
        char fc = flag[1] - 32; // 大写标志
        std::string flag2 = "-";
        flag2 += fc;
        if (strcmp(currArg, flag) == 0 || strcmp(currArg, flag2.c_str()) == 0)
        {
            if (++i >= argc)
            {
                printf("参数'%s'设置有误!请使用参数'--help'查看帮助!\n", currArg);
                exit(-1);
            }
            else
            {
                return 1;
            }
        }
        return 0;
    }
};

#endif