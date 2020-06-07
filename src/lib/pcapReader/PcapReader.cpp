#include <io/PcapReader.hpp>

PointCloudReader::PointCloudReader()
{
    inited = false;
    voxelLeafsize = 0;
}

void PointCloudReader::init(ArgParser args)
{
    if (!open_pcap()){
        printf("Cannot open pcap file:%s!\n", fileNamePcap.c_str());
        exit(-1);
    }

    if (reader.totalFrame() < minPcapSize){
        printf("The pcap data is too small!\n");
        exit(-1);
    }
    
    // 设置有效数据的范围 
    distanceControl = args.valid_dis;

    std::cout << getFileName(fileNamePcap)  << " has frames: " << reader.totalFrame() << std::endl;
    inited = true;
}

bool PointCloudReader::readPointCloud(PointCloud::Ptr _cloud, long long _frameID)
{
    if(!inited || !_cloud)
        return false;
    
    if(!reader.capture(frame, _frameID)){
        std::cout << "read frame " << _frameID << " failed." << std::endl;    
        return false;
    }

    _cloud->clear();
    for (int n = 0; n < frame.numLine; n++){
        for (int i = 0; i < frame.lines[n].num; i++){
            PointType pt;
            pt.x = (float)frame.lines[n].pPosition[i].x;
            pt.y = (float)frame.lines[n].pPosition[i].y;
            pt.z = (float)frame.lines[n].pPosition[i].z;
            float dist = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (distanceControl && (dist > distanceControl))
                continue;
            
            pt.intensity = frame.lines[n].pIntensity[i] + 0.1f;
            double timestamp = frame.lines[n].pTimestamp[i] / 1e6; //秒为单位
            pt.data_n[0] = int(timestamp);           //整数部分为秒
            pt.data_n[1] = timestamp - pt.data_n[0]; //微秒
            pt.data_n[2] = n;
            pt.data_n[3] = dist;
            _cloud->points.push_back(pt);
        }
    }
    _cloud->height = 1;
    _cloud->width = _cloud->points.size();
    _cloud->is_dense = true;
    
    //if(voxelLeafsize) voxel(_cloud);
    return true;
}

// void PointCloudReader::voxel(PointCloud::Ptr cloud)
// {
//     PointCloud::Ptr cloud_filtered(new PointCloud);
//     grid.setLeafSize(voxelLeafsize, voxelLeafsize, voxelLeafsize);
//     grid.setInputCloud(cloud);
//     grid.filter(*cloud_filtered);
//     *cloud = *cloud_filtered;

//     // remove NAN points from the cloud
//     std::vector<int> indices;
//     PointCloud::Ptr cloud_remove_nan(new PointCloud);
//     pcl::removeNaNFromPointCloud(*cloud, *cloud_remove_nan, indices);
//     *cloud = *cloud_remove_nan;
// }

bool PointCloudReader::open_pcap(){
    if (fileNamePcap == "" || calibrationPath == "")
        return false;
    return reader.open(fileNamePcap, calibrationPath);
}

