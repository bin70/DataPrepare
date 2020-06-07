#include <common.hpp>
#include <pcl/visualization/pcl_visualizer.h>

namespace vis_utils
{
    bool isPause = true;

    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                               void *nothing)
    {
        if (event.getKeySym() == "space" && event.keyDown())
            isPause = false;
    }

    void waitForSpace(pcl::visualization::PCLVisualizer *viewer)
    {
        isPause = true;
        while (isPause)
            viewer->spinOnce();
    }

    //以强度信息作为颜色显示点云
    inline void ShowCloud(const PointCloud::Ptr cloud,
                          pcl::visualization::PCLVisualizer *viewer,
                          std::string color_field = "intensity")
    {
        std::string cloud_id = "cloud_" + color_field;
        if (viewer->contains(cloud_id))
            viewer->removePointCloud(cloud_id);

        //从电云intensity字段生成颜色信息
        pcl::visualization::PointCloudColorHandlerGenericField<PointType> cloud_handle(cloud, color_field);
        viewer->setBackgroundColor(255, 255, 255);
        viewer->addPointCloud(cloud, cloud_handle, cloud_id);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_id);
        viewer->spinOnce();
    }

    inline void ShowCloud(const PointCloud::Ptr cloud,
                          pcl::visualization::PCLVisualizer *viewer,
                          int r, int g, int b)
    {
        std::stringstream ss;
        ss << "cloud_" << r << g << b;
        std::string cloud_id = ss.str();
        if (viewer->contains(cloud_id))
        {
            viewer->removePointCloud(cloud_id);
        }

        pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_handle(cloud, r, g, b);
        viewer->addPointCloud(cloud, cloud_handle, cloud_id);
        viewer->spinOnce();
    }
} // namespace vis_utils