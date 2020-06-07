#pragma once
#include <common.hpp>

void addConstraint(long long from, long long to, Eigen::Matrix4d tf, std::string _out_file)
{
    std::ofstream out(_out_file, std::ios::app);
    out << edge(from, to, tf) << std::endl;
    out.close();
}

std::string vertex(long long id, Eigen::Matrix4d m)
{
    Eigen::Vector3d pos = m.block(0, 3, 3, 1);
    Eigen::Matrix3d r = m.block(0, 0, 3, 3);
    Eigen::Quaterniond qua(r);
    std::string new_vertex = "VERTEX_SE3:QUAT " + std::to_string(id) + " ";
    new_vertex += (std::to_string(pos(0)) + " " + std::to_string(pos(1)) + " " + std::to_string(pos(2)) + " ");
    new_vertex += (std::to_string(qua.x()) + " " + std::to_string(qua.y()) + " " + std::to_string(qua.z()) + " " + std::to_string(qua.w()));
    return new_vertex;
}

std::string edge(long long from, long long to, Eigen::Matrix4d deltaH)
{
    Eigen::Vector3d pos = deltaH.block(0, 3, 3, 1);
    Eigen::Matrix3d r = deltaH.block(0, 0, 3, 3);
    Eigen::Quaterniond qua(r);
    std::string edge = "EDGE_SE3:QUAT " + std::to_string(from) + " " + std::to_string(to) + " " + std::to_string(pos(0)) + " " + std::to_string(pos(1)) + " " + std::to_string(pos(2)) + " " + std::to_string(qua.x()) + " " + std::to_string(qua.y()) + " " + std::to_string(qua.z()) + " " + std::to_string(qua.w()) + " " + "10000 0 0 0 0 0 10000 0 0 0 0 10000 0 0 0 40000 0 0 40000 0 40000";
    return edge;
}