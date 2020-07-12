#pragma once
#include <utility>
#include <time.h>
#include <iomanip>
#include <map>
#include <common.hpp>
#include <loam/transform.hpp>

#define UNUSED_LINES 3

enum TrajType{
    ROS_LOAM, G2O, KITTI
};

class PoseNode
{
public:
    long long frameID;
    Eigen::Matrix4d tf;
    double timestamp;
    Eigen::Quaterniond qua;
    Eigen::Vector3d pos;
    Eigen::Matrix<double, 6, 1> loamTrans;
    std::vector<double> loamTrans2;

    PoseNode() : frameID(0), timestamp(0), //label(false),
                 tf(Eigen::Matrix4d::Identity())
    {
        loamTrans2.resize(12);
    }

    PoseNode operator=(const PoseNode &value)
    {
        this->frameID = value.frameID;
        this->timestamp = value.timestamp;
        //this->label = value.label;
        this->tf = value.tf;
        return *this;
    }
};

class TrajIO
{
public:
    TrajIO(std::string traj_path, TrajType type = ROS_LOAM)
    :   startID(-1),
        endID(-1),
        frameGap(0),
        data_name(getFileName(traj_path))
    {
        assert( CheckFileExist(traj_path.c_str()) == true );
        traj_file_.open(traj_path.c_str());

        // 头文件
        for (int i = 0; i < UNUSED_LINES && !traj_file_.eof(); ++i)
            getline(traj_file_, unused);

        // 读取开始帧号
        if(readPose(type))
            startID = curPose.frameID;

        assert(startID >=0);

        // 获取帧间隔
        if(readPose(type))
            frameGap = curPose.frameID - startID;
        
        assert(frameGap>=1 && frameGap<=3);

        // 读取剩余位姿
        while(readPose(type))
            ;
        
        traj_file_.close();
        endID = curPose.frameID;
        
        std::cout << "\n======================== Details of trajectory ============================" << std::endl
                  << "\tstartID in trajectory = " << startID << std::endl
                  << "\tendID in trajectory = " << endID << std::endl
                  << "\tframeGap = " << frameGap << std::endl
                  << "\tframe number = " << traj.size() << std::endl
                  << "==============================================================================" << std::endl;
    }
        
    
    PoseNode getPose(int frameID)
    {
        assert(traj.count(frameID) > 0);
        return traj[frameID];
    }

    const int getFrameGap() { return frameGap; }
    const int getStartID() { return startID;}
    const int getEndID(){ return endID;}
    Eigen::Matrix4d getPoseMatrix(int frameID){ return getPose(frameID).tf;}

    void writeToFile(std::string out_file)
    {
        assert( CheckFileExist(out_file.c_str()) == true);
        
        std::ofstream out(out_file);
        if(!out.is_open() || out.fail())
            throw std::ios_base::failure("Error in opening file: " + out_file );

        writeHeader(out);

        for (int i = startID; i < traj.size(); i += frameGap)
            writePose(out, traj[i].frameID);
        
        traj_file_.close();
    }
    
    bool readPose(TrajType type)
    {
        switch(type)
        {
            case ROS_LOAM:
                return readPoseLOAM();
            case G2O:
                return readPoseG2O();
            case KITTI:
                return readPoseKITTI();
        }
    }

    //写入轨迹
    void writePose(std::ofstream &out, int frameID, std::string label = "ROS_LOAM_POSE")
    {
        double time = getPose(frameID).timestamp;
        out << label << ' ' << frameID << ' ';

        if(label == "VERTEX_SE3")
        {
            Eigen::Matrix4d tf = getPoseMatrix(frameID);
            Eigen::Matrix3d r = tf.block(0, 0, 3, 3);
            Eigen::Vector3d v = tf.block(0, 3, 3, 1);
            Eigen::Quaterniond q(r);
            out << v(0) << ' ' << v(1) << ' ' << v(2) << ' '
                << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << ' '
                << time;
        }
        else if(label == "ROS_LOAM_POSE")
        {
            Eigen::Matrix<double, 6, 1> poseVec = getPose(frameID).loamTrans;
            for(int i=0; i<6; ++i)
                out << poseVec[i];
            out << time;
        }
    }

    void writeHeader(std::ofstream &out, std::string label = "ROS_LOAM_POSE")
    {
        time_t now = time(NULL);
        out << "# DataName:\t" << data_name.c_str() << std::endl
            << "# Date:\t\t" << ctime(&now)
            << "# Data Format: {" << label;
        if(label == "VERTEX_SE3" || label == "iSAM_VERTEX")
            out << ", FrameID, Position[x|y|z], Quaternion[qx|qy|qz|qw],Timestamp(us)}" << std::endl;
        else if(label == "ROS_LOAM_POSE")
            out << ", FrameID, Euler[x|y|z], Position[x|y|z], Timestamp(us)}" << std::endl;
    }

private:
    std::map<int, PoseNode> traj;
    bool readPoseLOAM()
    {
        if(!(traj_file_ >> unused)) return false;
        
        traj_file_ >> curPose.frameID;
        for (int i = 0; i < 6; ++i)
            traj_file_ >> curPose.loamTrans[i];
        traj_file_ >> curPose.timestamp;

        curPose.tf = loam::euler2matrix(curPose.loamTrans);
        Eigen::Matrix3d r = curPose.tf.block(0, 0, 3, 3);
        Eigen::Quaterniond q(r);
        curPose.qua = q;
        curPose.pos = curPose.tf.block(0, 3, 3, 1);

        traj[curPose.frameID] = curPose;
        return true;
    }

    bool readPoseG2O()
    {
        if(!(traj_file_ >> unused)) return false;

        Eigen::Quaterniond q;
        Eigen::Vector3d v;
        traj_file_ >> curPose.frameID >> v(0) >> v(1) >> v(2) >> q.x() >> q.y() >> q.z() >> q.w() >> curPose.timestamp;
        curPose.qua = q;
        curPose.pos = v;
        curPose.tf = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d r = q.toRotationMatrix();
        curPose.tf.block(0, 0, 3, 3) = r;
        curPose.tf.block(0, 3, 3, 1) = v;

        traj[curPose.frameID] = curPose;

        return true;
    }

    bool readPoseKITTI()
    {
        if(traj_file_.eof()) return false;
        
        std::string line;
        std::vector<std::string> st;

        getline(traj_file_, line);
        if(line.empty()) return false;

        boost::trim(line);
        boost::split(st, line, boost::is_any_of("\r\t "), boost::token_compress_on);

        if(st.size() != 12)
        {
            std::cout << "轨迹格式有问题" << std::endl;
            return false;
        }
        
        curPose.tf = Eigen::Matrix4d::Identity();

        for(int row=0; row<3; ++row)
        {
            for(int col=0; col<4; ++col)
            {
                curPose.tf(row, col) = atof(st[row*4+col].c_str());
            }
        }
        
        Eigen::Matrix3d r = curPose.tf.block(0, 0, 3, 3);
        Eigen::Quaterniond q(r);
        curPose.qua = q;
        curPose.pos = curPose.tf.block(0, 3, 3, 1);

        traj[curPose.frameID] = curPose;
    }

private:
    std::string unused;
    long long startID;
    long long endID;
    int frameGap;
    PoseNode curPose;
    std::fstream traj_file_;
    std::string data_name;

};