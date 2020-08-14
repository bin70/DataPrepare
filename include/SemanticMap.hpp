#pragma once
#include <constant/label.h>
#include <utils/common.hpp>
#include <point_cloud/TXTReader.hpp>

class SemanticMap
{
    using string = std::string;

public:
    std::map<int, string> label_name = {
        {CEILING, "ceiling"},
        {FLOOR, "floor"},
        {WALL,  "wall"},
        {CLUTTER, "clutter"}
    };

public:
    SemanticMap(string map_folder, float resolution = 0.3) :
        _resolution(resolution),
        _map_folder(map_folder),
        _cloud(new PointCloud),
        _map_manager(new MapManager(_resolution))
    {
        std::cout << "Loading semantic map ..." << std::endl;
        
        loadMap();

        std::cout << "Loaded semantic map:" << std::endl
                  << "\t - resolution = " << _resolution << std::endl
                  << "\t - size = " << getMapPtr()->points.size() << std::endl;
        pLine();
    }

    bool loadMap()
    {
        reader.openDir(_map_folder);
        reader.setDataCol(3);

        for (int label = 0; label < CLUTTER; ++label)
        {
            reader.readPointCloud(_cloud, label_name[label]);
            for(auto &p : _cloud->points)
            {
                p.curvature = label;
            }
            
            // _map_manager->getMapPtr()->points.push_back(p);

            if(label == 0)
                _map_manager->UpdateMap(_cloud);
            else
                _map_manager->AddFrameToMap(_cloud);
                
            consoleProgress(100/(label_name.size()-1)*(label+1));
        }

        //_map_manager->update();
        _cloud->clear();
        return true;
    }
    bool searchPoint(PointType& query, PointType& result)
    {
        return _map_manager->getNearestPoint(query, result);
    }

    const PointCloud::Ptr& getMapPtr(){ return _map_manager->getMapPtr(); }

private:
    TXTReader reader;
    FileOperator fop;
    float _resolution;
    string _map_folder;
    PointCloud::Ptr _cloud;
    MapManager* _map_manager;
};