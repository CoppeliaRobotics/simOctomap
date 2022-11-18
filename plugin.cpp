#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <utility>
#include <algorithm>

#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_utility.hpp>

#include "simPlusPlus/Plugin.h"
#include "simPlusPlus/Handle.h"
#include "config.h"
#include "plugin.h"
#include "stubs.h"

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/math/Utils.h>

#define PROXIMITY_SENSOR_INFLATE 0.001

using sim::Handle;
using sim::Handles;
using OcTree = octomap::ColorOcTree;
using OcTreeKey = octomap::OcTreeKey;
using OcTreeNode = octomap::ColorOcTreeNode;

template<> std::string Handle<OcTree>::tag() { return "octomap.OcTree"; }
template<> std::string Handle<OcTreeNode>::tag() { return "octomap.OcTreeNode"; }

struct Vertex
{
    octomap::point3d coord;
    const OcTreeNode *node;
};

struct Edge
{
    double weight;
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Vertex, Edge> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor VertexDesc;
typedef boost::graph_traits<Graph>::edge_descriptor EdgeDesc;
typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;

struct GraphContainer
{
    std::map<const OcTreeNode*, VertexDesc> revmap;
    Graph graph;
    double minWeight = 1.0;
    double maxWeight = 2.0;

    VertexDesc vertexAt(const OcTreeNode *node)
    {
        return revmap.at(node);
    }

    void addVertex(const OcTreeNode &node, const octomap::point3d &point)
    {
        VertexDesc vd = boost::add_vertex(graph);
        graph[vd].coord = point;
        graph[vd].node = &node;
        revmap[&node] = vd;
    }

    void addEdge(const OcTreeNode &node1, const OcTreeNode &node2, double weight)
    {
        VertexDesc v1 = revmap.at(&node1), v2 = revmap.at(&node2);
        if(v1 == v2) return;
        EdgeDesc e; bool added;
        boost::tie(e, added) = boost::add_edge(v1, v2, graph);
        graph[e].weight = weight;
        maxWeight = fmax(maxWeight, weight);
    }
};
template<> std::string Handle<GraphContainer>::tag() { return "Graph"; }

class Plugin : public sim::Plugin
{
public:
    void onStart()
    {
        if(!registerScriptStuff())
            throw std::runtime_error("failed to register script stuff");

        setExtVersion("Octomap Plugin");
        setBuildDate(BUILD_DATE);
    }

    void onScriptStateDestroyed(int scriptID)
    {
        for(auto obj : treeHandles.find(scriptID))
            delete treeHandles.remove(obj);
        for(auto obj : graphContainerHandles.find(scriptID))
            delete graphContainerHandles.remove(obj);
    }

    void create(create_in *in, create_out *out)
    {
        OcTree *octree = new OcTree(in->resolution);
        out->octreeHandle = treeHandles.add(octree, in->_.scriptID);
    }

    int createProximitySensor(float size)
    {
        int options = 0 // sensor options:
                +1*1   // the sensor will be explicitely handled
                +1*2   // the detection volumes are not shown when detecting something
                +1*4   // the detection volumes are not shown when not detecting anything
                +0*8   // front faces are not detected
                +0*16  // back faces are not detected
                +1*32  // fast detection (i.e. not exact detection)
                +0*64  // the normal of the detected surface with the detection ray will have to lie below a specified threshold angle
                +0*128 // occlusion check is active
                +0*256 // smallest distance threshold will be active
                +0*512;// randomized detection (only with ray-type proximity sensors)
        int intParams[] = {
                0,     // face count (volume description)
                0,     // face count far (volume description)
                0,     // subdivisions (volume description)
                0,     // subdivisions far (volume description)
                0,     // randomized detection, sample count per reading
                0,     // randomized detection, individual ray detection count for triggering
                0,     // reserved. Set to 0
                0      // reserved. Set to 0
        };
        float floatParams[] = {
                0.0,   // offset (volume description)
                size,  // range (volume description)
                size,  // x size (volume description)
                size,  // y size (volume description)
                size,  // x size far (volume description)
                size,  // y size far (volume description)
                0.0,   // inside gap (volume description)
                0.0,   // radius (volume description)
                0.0,   // radius far (volume description)
                0.0,   // angle (volume description)
                0.0,   // threshold angle for limited angle detection (see bit 6 above)
                0.0,   // smallest detection distance (see bit 8 above)
                0.0,   // sensing point size
                0.0,   // reserved. Set to 0.0
                0.0    // reserved. Set to 0.0
        };
        return simCreateProximitySensor(sim_proximitysensor_pyramid_subtype, sim_objectspecialproperty_detectable_all, options, intParams, floatParams, NULL);
    }

    octomap::point3d snapCoord(OcTree *octree, int depth, octomap::point3d coord)
    {
        OcTreeKey key = octree->coordToKey(coord, depth);
        octomap::point3d coord1 = octree->keyToCoord(key, depth);
        return coord1;
    }

    void measureOccupancy(OcTree *octree, int depth, octomap::point3d coord, const octomap::point3d& boundsMin, const octomap::point3d& boundsMax, std::map<int,int>& proximitySensors)
    {
        double nodeSize = octree->getNodeSize(depth);

        // check if the voxel being measured is completely out of bounds, and in that case skip:
        if(coord.x() + 0.5 * nodeSize < boundsMin.x() || coord.x() - 0.5 * nodeSize > boundsMax.x()
                || coord.y() + 0.5 * nodeSize < boundsMin.y() || coord.y() - 0.5 * nodeSize > boundsMax.y()
                || coord.z() + 0.5 * nodeSize < boundsMin.z() || coord.z() - 0.5 * nodeSize > boundsMax.z())
            return;

        if(proximitySensors.find(depth) == proximitySensors.end())
        {
            proximitySensors[depth] = createProximitySensor(nodeSize + PROXIMITY_SENSOR_INFLATE);
        }
        int sens = proximitySensors[depth];

        float p[4];
        p[0] = coord.x();
        p[1] = coord.y();
        p[2] = coord.z() - nodeSize * 0.5 - PROXIMITY_SENSOR_INFLATE * 0.5; // proximity sensor's origin is in its XY plane!
        simSetObjectPosition(sens, -1, &p[0]);
        int r = simCheckProximitySensor(sens, sim_handle_all, &p[0]);

        if(r == 1) // some obstacle detected
        {
            if(depth == octree->getTreeDepth()) // maximum depth -> single update
            {
                octree->updateNode(coord, true);
            }
            else // recursive update
            {
                int depth1 = depth + 1;
                double nodeSize1 = octree->getNodeSize(depth1);
                double off1 = 0.5 * (nodeSize - nodeSize1);
                measureOccupancy(octree, depth1, coord + octomap::point3d(-off1, -off1, -off1), boundsMin, boundsMax, proximitySensors);
                measureOccupancy(octree, depth1, coord + octomap::point3d(+off1, -off1, -off1), boundsMin, boundsMax, proximitySensors);
                measureOccupancy(octree, depth1, coord + octomap::point3d(-off1, +off1, -off1), boundsMin, boundsMax, proximitySensors);
                measureOccupancy(octree, depth1, coord + octomap::point3d(+off1, +off1, -off1), boundsMin, boundsMax, proximitySensors);
                measureOccupancy(octree, depth1, coord + octomap::point3d(-off1, -off1, +off1), boundsMin, boundsMax, proximitySensors);
                measureOccupancy(octree, depth1, coord + octomap::point3d(+off1, -off1, +off1), boundsMin, boundsMax, proximitySensors);
                measureOccupancy(octree, depth1, coord + octomap::point3d(-off1, +off1, +off1), boundsMin, boundsMax, proximitySensors);
                measureOccupancy(octree, depth1, coord + octomap::point3d(+off1, +off1, +off1), boundsMin, boundsMax, proximitySensors);
            }
        }
        else if(r == 0) // no obstacle detected -> mark area as empty
        {
            int maxDepth = octree->getTreeDepth();
            octomap::point3d boundsMinSnap = snapCoord(octree, maxDepth, boundsMin);
            octomap::point3d boundsMaxSnap = snapCoord(octree, maxDepth, boundsMax);
            double res = octree->getResolution();
            double off = 0.5 * (nodeSize - res);
            double minX = fmax(coord.x() - off, boundsMinSnap.x());
            double maxX = fmin(coord.x() + off, boundsMaxSnap.x());
            double minY = fmax(coord.y() - off, boundsMinSnap.y());
            double maxY = fmin(coord.y() + off, boundsMaxSnap.y());
            double minZ = fmax(coord.z() - off, boundsMinSnap.z());
            double maxZ = fmin(coord.z() + off, boundsMaxSnap.z());

            for(double z = minZ; z <= maxZ; z += res)
            {
                for(double y = minY; y <= maxY; y += res)
                {
                    for(double x = minX; x <= maxX; x += res)
                    {
                        octree->updateNode(x, y, z, false);
                    }
                }
            }
        }
    }

    void createFromScene(createFromScene_in *in, createFromScene_out *out)
    {
        if(in->boundsMin.size() != 3 || in->boundsMax.size() != 3)
            throw std::runtime_error("bounds must have 3 elements");

        for(int i = 0; i < 3; i++)
            if(in->boundsMin[i] >= in->boundsMax[i])
                throw std::runtime_error("bounds min must be strictly lower than max");

        OcTree *octree = new OcTree(in->resolution);

#ifndef DISABLE_FAST_OCCUPANCY_MEASUREMENT
        // snap bounds to voxel coords at minimum depth:
        int depth = 6;
        octomap::point3d boundsMin(in->boundsMin[0], in->boundsMin[1], in->boundsMin[2]);
        octomap::point3d boundsMax(in->boundsMax[0], in->boundsMax[1], in->boundsMax[2]);
        octomap::point3d boundsMinSnap = snapCoord(octree, depth, boundsMin);
        octomap::point3d boundsMaxSnap = snapCoord(octree, depth, boundsMax);
        double nodeSize = octree->getNodeSize(depth);

        // CoppeliaSim's proximity sensors, by depth:
        std::map<int,int> proximitySensors;

        for(double z = boundsMinSnap.z(); z <= boundsMaxSnap.z(); z += nodeSize)
        {
            for(double y = boundsMinSnap.y(); y <= boundsMaxSnap.y(); y += nodeSize)
            {
                for(double x = boundsMinSnap.x(); x <= boundsMaxSnap.x(); x += nodeSize)
                {
                    measureOccupancy(octree, depth, octomap::point3d(x, y, z), boundsMin, boundsMax, proximitySensors);
                }
            }
        }

        for(std::map<int,int>::iterator it = proximitySensors.begin(); it != proximitySensors.end(); ++it)
            simRemoveObject(it->second);

#else
        int sens = createProximitySensor(in->resolution + PROXIMITY_SENSOR_INFLATE);

        for(float z = in->boundsMin[2]; z <= in->boundsMax[2]; z += in->resolution)
        {
            for(float y = in->boundsMin[1]; y <= in->boundsMax[1]; y += in->resolution)
            {
                for(float x = in->boundsMin[0]; x <= in->boundsMax[0]; x += in->resolution)
                {
                    float p[4];
                    p[0] = x;
                    p[1] = y;
                    p[2] = z - 0.5 * in->resolution - 0.5 * PROXIMITY_SENSOR_INFLATE;
                    simSetObjectPosition(sens, -1, &p[0]);
                    int r = simCheckProximitySensor(sens, sim_handle_all, &p[0]);
                    bool occ = r == 1;
                    octree->updateNode(x, y, z, occ);
                }
            }
        }

        simRemoveObject(sens);
#endif

        out->octreeHandle = treeHandles.add(octree, in->_.scriptID);
    }

    void destroy(destroy_in *in, destroy_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        delete octree;
    }

    octomap::point3d vectorToPoint(std::vector<float> v)
    {
        return octomap::point3d(v[0], v[1], v[2]);
    }

    OcTreeKey vectorToKey(std::vector<int> v)
    {
        OcTreeKey key;
        key[0] = v[0];
        key[1] = v[1];
        key[2] = v[2];
        return key;
    }

    std::vector<float> pointToVector(octomap::point3d p)
    {
        std::vector<float> v;
        v.push_back(p.x());
        v.push_back(p.y());
        v.push_back(p.z());
        return v;
    }

    std::vector<int> keyToVector(OcTreeKey k)
    {
        std::vector<int> v;
        v.push_back(k[0]);
        v.push_back(k[1]);
        v.push_back(k[2]);
        return v;
    }

    void clear(clear_in *in, clear_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        octree->clear();
    }

    void coordToKey(coordToKey_in *in, coordToKey_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        octomap::point3d coord = vectorToPoint(in->coord);
        OcTreeKey key = octree->coordToKey(coord);
        out->key = keyToVector(key);
    }

    void keyToCoord(keyToCoord_in *in, keyToCoord_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        OcTreeKey key = vectorToKey(in->key);
        octomap::point3d coord = octree->keyToCoord(key);
        out->coord = pointToVector(coord);
    }

    void deleteNode(deleteNode_in *in, deleteNode_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        octomap::point3d coord = vectorToPoint(in->coord);
        octree->deleteNode(coord, in->depth);
    }

    void deleteNodeWithKey(deleteNodeWithKey_in *in, deleteNodeWithKey_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        OcTreeKey key = vectorToKey(in->key);
        octree->deleteNode(key, in->depth);
    }

    void getMetricBounds(getMetricBounds_in *in, getMetricBounds_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        double minx, miny, minz, maxx, maxy, maxz, sizex, sizey, sizez;
        octree->getMetricMin(minx, miny, minz);
        octree->getMetricMax(maxx, maxy, maxz);
        octree->getMetricSize(sizex, sizey, sizez);
        out->boundsMin = {float(minx), float(miny), float(minz)};
        out->boundsMax = {float(maxx), float(maxy), float(maxz)};
        out->size = {float(sizex), float(sizey), float(sizez)};
    }

    void getNodeSize(getNodeSize_in *in, getNodeSize_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        out->size = octree->getNodeSize(in->depth);
    }

    void getNumLeafNodes(getNumLeafNodes_in *in, getNumLeafNodes_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        out->n = octree->getNumLeafNodes();
    }

    void getSize(getSize_in *in, getSize_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        out->size = octree->size();
    }

    void getVolume(getVolume_in *in, getVolume_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        out->volume = octree->volume();
    }

    void getResolution(getResolution_in *in, getResolution_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        out->res = octree->getResolution();
    }

    void getTreeDepth(getTreeDepth_in *in, getTreeDepth_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        out->depth = octree->getTreeDepth();
    }

    void getTreeType(getTreeType_in *in, getTreeType_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        out->treeType = octree->getTreeType();
    }

    void prune(prune_in *in, prune_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        octree->prune();
    }

    void updateNode(updateNode_in *in, updateNode_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        octomap::point3d coord = vectorToPoint(in->coord);
        switch(in->mode)
        {
        case sim_octomap_update_log_odds:
            octree->updateNode(coord, in->log_odds);
            break;
        case sim_octomap_update_occupancy:
            octree->updateNode(coord, in->occupancy);
            break;
        default:
            throw std::runtime_error("invalid update mode");
        }
    }

    void updateNodeWithKey(updateNodeWithKey_in *in, updateNodeWithKey_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        OcTreeKey key = vectorToKey(in->key);
        switch(in->mode)
        {
        case sim_octomap_update_log_odds:
            octree->updateNode(key, in->log_odds);
            break;
        case sim_octomap_update_occupancy:
            octree->updateNode(key, in->occupancy);
            break;
        default:
            throw std::runtime_error("invalid update mode");
        }
    }

    void insertPointCloud(insertPointCloud_in *in, insertPointCloud_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");

        if(!in->points.size()) return;

        if(in->origin.size() != 3)
            throw std::runtime_error("origin must be a table of size 3");

        if(in->points.size() % 3)
            throw std::runtime_error("points size is not a multiple of 3");

        int n = in->points.size() / 3;
        octomap::Pointcloud cloud;
        for(int i = 0; i < n; i++)
            cloud.push_back(in->points[i*3+0], in->points[i*3+1], in->points[i*3+2]);
        octomap::point3d origin = vectorToPoint(in->origin);
        octomap::pose6d frame_origin(0, 0, 0, 0, 0, 0);
        octree->insertPointCloud(cloud, origin, frame_origin, in->maxRange);
    }

    void castRay(castRay_in *in, castRay_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");

        if(in->origin.size() != 3)
            throw std::runtime_error("origin must be a table of size 3");

        if(in->direction.size() != 3)
            throw std::runtime_error("direction must be a table of size 3");

        octomap::point3d origin = vectorToPoint(in->origin);
        octomap::point3d direction = vectorToPoint(in->direction);
        octomap::point3d end;
        out->hit = octree->castRay(origin, direction, end, in->ignoreUnknownCells, in->maxRange);
        out->end = pointToVector(end);
    }

    void write(write_in *in, write_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        octree->write(in->filename);
    }

    void writeBinary(writeBinary_in *in, writeBinary_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        octree->writeBinary(in->filename);
    }

    void valueColor(float v, float vmin, float vmax, float &r, float &g, float &b)
    {
        r = g = b = 1.0;
        if(v < vmin) v = vmin;
        if(v > vmax) v = vmax;
        float dv = vmax - vmin;
        if(v < (vmin + 0.25 * dv)) {
            r = 0;
            g = 4 * (v - vmin) / dv;
        } else if(v < (vmin + 0.5 * dv)) {
            r = 0;
            b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
        } else if(v < (vmin + 0.75 * dv)) {
            r = 4 * (v - vmin - 0.5 * dv) / dv;
            b = 0;
        } else {
            g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
            b = 0;
        }
    }

    void valueColor(float value, float &r, float &g, float &b)
    {
        valueColor(value, 0.0, 1.0, r, g, b);
    }

    void addDrawingObject(addDrawingObject_in *in, addDrawingObject_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");

        if(in->voxelColor == sim_octomap_voxelcolor_flat && in->flatColor.size() != 3)
            throw std::runtime_error("flatColor must have 3 elements");

        double minX, minY, minZ, sizeX, sizeY, sizeZ;
        octree->getMetricMin(minX, minY, minZ);
        octree->getMetricSize(sizeX, sizeY, sizeZ);

        int handle = simAddDrawingObject(sim_drawing_cubepoints + sim_drawing_itemcolors + sim_drawing_itemsizes, 0, 0.0, -1, 1000000, NULL, NULL, NULL, NULL);
        float data[10];

        OcTree::leaf_iterator begin = octree->begin(in->depth), end = octree->end();
        for(OcTree::leaf_iterator it = begin; it != end; ++it)
        {
            if(!octree->isNodeOccupied(*it) && in->skipFree) continue;
            // cube position:
            data[0] = it.getX();
            data[1] = it.getY();
            data[2] = it.getZ();
            // normal:
            data[3] = 0;
            data[4] = 0;
            data[5] = 1;
            // color:
            switch(in->voxelColor)
            {
                case sim_octomap_voxelcolor_flat:
                    data[6] = in->flatColor[0];
                    data[7] = in->flatColor[1];
                    data[8] = in->flatColor[2];
                    break;
                case sim_octomap_voxelcolor_x_axis:
                    valueColor((it.getX() - minX) / sizeX, data[6], data[7], data[8]);
                    break;
                case sim_octomap_voxelcolor_y_axis:
                    valueColor((it.getY() - minY) / sizeY, data[6], data[7], data[8]);
                    break;
                case sim_octomap_voxelcolor_z_axis:
                    valueColor((it.getZ() - minZ) / sizeZ, data[6], data[7], data[8]);
                    break;
                case sim_octomap_voxelcolor_value:
                    valueColor(it->getValue(), data[6], data[7], data[8]);
                    break;
            }
            // size:
            data[9] = it.getSize() * 0.5;

            int ret = simAddDrawingObjectItem(handle, &data[0]);
            if(ret != 1)
            {
                // TODO: set error
                break;
            }
        }

        out->handle = handle;
    }

    void isOccupied(isOccupied_in *in, isOccupied_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        octomap::point3d coord = vectorToPoint(in->coord);
        OcTreeNode *node = octree->search(coord, in->depth);
        if(!node)
        {
            out->occupancy = -1;
        }
        else
        {
            out->occupancy = octree->isNodeOccupied(node) ? 1 : 0;
        }
    }

    void isOccupiedKey(isOccupiedKey_in *in, isOccupiedKey_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        OcTreeKey key = vectorToKey(in->key);
        OcTreeNode *node = octree->search(key, in->depth);
        if(!node)
        {
            out->occupancy = -1;
        }
        else
        {
            out->occupancy = octree->isNodeOccupied(node) ? 1 : 0;
        }
    }

    double localZMoment(OcTree *octree, octomap::point3d p, octomap::point3d radius = octomap::point3d(2, 2, 4), int order = 1, double mean = 0.0)
    {
        double sum = 0;
        int count = 0;
        double res = octree->getResolution();
        OcTree::leaf_bbx_iterator begin1 = octree->begin_leafs_bbx(p - radius * res, p + radius * res),
            end1 = octree->end_leafs_bbx();
        for(OcTree::leaf_bbx_iterator it1 = begin1; it1 != end1; ++it1)
        {
            sum += pow(it1.getCoordinate().z() - mean, order);
            count++;
        }
        return sum / count;
    }

    double borderPenalty(OcTree *octree, octomap::point3d p, octomap::point3d radius = octomap::point3d(1, 1, 3))
    {
        int count = 0;
        double res = octree->getResolution();
        OcTree::leaf_bbx_iterator begin1 = octree->begin_leafs_bbx(p - radius * res, p + radius * res),
            end1 = octree->end_leafs_bbx();
        for(OcTree::leaf_bbx_iterator it1 = begin1; it1 != end1; ++it1)
        {
            count++;
        }
        return pow(9 - count, 2);
    }

    void computeCostmap(OcTree *octree, octomap::point3d radius = octomap::point3d(2, 2, 4))
    {
        OcTree::leaf_iterator begin = octree->begin_leafs(),
            end = octree->end_leafs();
        double v, vmin, vmax;
        bool first = true;
        for(OcTree::leaf_iterator it = begin; it != end; ++it)
        {
            octomap::point3d c = it.getCoordinate();
            double mean = localZMoment(octree, c, radius, 1, 0.0);
            double var = localZMoment(octree, c, radius, 2, mean);
            double bp = borderPenalty(octree, c);
            v = var + 0.5 * bp;
            vmin = first ? v : fmin(v, vmin);
            vmax = first ? v : fmax(v, vmax);
            first = false;
            it->setValue(v);
        }
        for(OcTree::leaf_iterator it = begin; it != end; ++it)
        {
            it->setValue(1 * (it->getValue() - vmin + 0.01) / (vmax - vmin + 0.01));
        }
    }

    double costAverage(OcTree *octree, octomap::point3d p, octomap::point3d radius)
    {
        double sum = 0;
        int count = 0;
        double res = octree->getResolution();
        OcTree::leaf_bbx_iterator begin1 = octree->begin_leafs_bbx(p - radius * res, p + radius * res),
            end1 = octree->end_leafs_bbx();
        for(OcTree::leaf_bbx_iterator it1 = begin1; it1 != end1; ++it1)
        {
            sum += it1->getValue();
            count++;
        }
        return sum / count;
    }

    void smoothCostmap(OcTree *octree, octomap::point3d radius = octomap::point3d(1, 1, 3))
    {
        OcTree::leaf_iterator begin = octree->begin_leafs(),
            end = octree->end_leafs();
        for(OcTree::leaf_iterator it = begin; it != end; ++it)
        {
            octomap::point3d c = it.getCoordinate();
            double avg = costAverage(octree, c, radius);
            it->setValue(avg);
        }
    }

    void f(f_in *in, f_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        octree->expand();

        OcTree *gnd = new OcTree(octree->getResolution());
        out->gndOctreeHandle = treeHandles.add(gnd, in->_.scriptID);

        octomap::point3d rxyz(in->s, in->s, in->h), rxy(in->s, in->s, 0), rz(0, 0, in->h), rz0(0, 0, octree->getResolution());

        OcTree::leaf_iterator begin = octree->begin_leafs(),
            end = octree->end_leafs();
        for(OcTree::leaf_iterator it = begin; it != end; ++it)
        {
            if(!octree->isNodeOccupied(*it)) continue;
            octomap::point3d c = it.getCoordinate();
            bool ok = true;

            // this check is redundant but make it faster
            OcTree::leaf_bbx_iterator begin1 = octree->begin_leafs_bbx(c + rz0, c + rz),
                end1 = octree->end_leafs_bbx();
            for(OcTree::leaf_bbx_iterator it1 = begin1; it1 != end1; ++it1)
            {
                if(octree->isNodeOccupied(*it1)) {ok=false; break;}
            }
            if(!ok) continue;

            // this is the actual check
            OcTree::leaf_bbx_iterator begin2 = octree->begin_leafs_bbx(c - rxy, c + rxyz),
                end2 = octree->end_leafs_bbx();
            for(OcTree::leaf_bbx_iterator it2 = begin2; it2 != end2; ++it2)
            {
                octomap::point3d q = it2.getCoordinate() - c;
                if(q.z() <= 0.5*octree->getResolution()) continue;
                if(sqrt(q.x()*q.x()+q.y()*q.y()) < in->s*q.z()/in->h && octree->isNodeOccupied(*it2)) {ok = false; break;}
            }
            if(!ok) continue;

            gnd->updateNode(c, true);
        }
        computeCostmap(gnd);
        for(int i = 0; i < 8; i++) smoothCostmap(gnd);
        octree->prune();
    }

    void createGraph(createGraph_in *in, createGraph_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");

        // build vertex set
        GraphContainer *g = new GraphContainer();
        OcTree::leaf_iterator begin = octree->begin_leafs(),
            end = octree->end_leafs();
        for(OcTree::leaf_iterator it = begin; it != end; ++it)
            //if(octree->isNodeOccupied(*it))
                g->addVertex(*it, it.getCoordinate());

        // add edges:
        float res = octree->getResolution();
        const octomap::point3d ri(in->connRadius[0], in->connRadius[1], in->connRadius[2]);
        const octomap::point3d r = ri * res;
        for(OcTree::leaf_iterator it = begin; it != end; ++it)
        {
            //if(!octree->isNodeOccupied(*it)) continue;
            octomap::point3d coord = it.getCoordinate();
            OcTree::leaf_bbx_iterator begin1 = octree->begin_leafs_bbx(coord - r, coord + r),
                end1 = octree->end_leafs_bbx();
            for(OcTree::leaf_bbx_iterator it1 = begin1; it1 != end1; ++it1)
            {
                //if(!octree->isNodeOccupied(*it1)) continue;
                octomap::point3d coord1 = it1.getCoordinate();
                double weight = in->weightConst;
                weight += in->weightDist * coord.distance(coord1);
                weight += in->weightSqDist[0] * pow(coord.x() - coord1.x(), 2);
                weight += in->weightSqDist[1] * pow(coord.y() - coord1.y(), 2);
                weight += in->weightSqDist[2] * pow(coord.z() - coord1.z(), 2);
                weight += 100 * it1->getValue();
                g->addEdge(*it, *it1, weight);
            }
        }

        out->graphHandle = graphContainerHandles.add(g, in->_.scriptID);
    }

    void addGraphDrawingObject(addGraphDrawingObject_in *in, addGraphDrawingObject_out *out)
    {
        GraphContainer *g = graphContainerHandles.get(in->graphHandle);

        float color[] = {1.0, 0.0, 0.0};
        int handle = simAddDrawingObject(sim_drawing_lines + sim_drawing_itemcolors, 3, 0.0, -1, 1000000, &color[0], NULL, NULL, NULL);
        float data[10];

        EdgeIter ei, ei_end;
        for(boost::tie(ei, ei_end) = boost::edges(g->graph); ei != ei_end; ++ei)
        {
            VertexDesc v0 = boost::source(*ei, g->graph),
                       v1 = boost::target(*ei, g->graph);
            octomap::point3d p0 = g->graph[v0].coord,
                p1 = g->graph[v1].coord;
            // first segment position:
            data[0] = p0.x();
            data[1] = p0.y();
            data[2] = p0.z();
            // second segment position:
            data[3] = p1.x();
            data[4] = p1.y();
            data[5] = p1.z();
            // color:
            double w = (g->graph[*ei].weight - g->minWeight) / (g->maxWeight - g->minWeight);
            valueColor(w, data[6], data[7], data[8]);
            // size:
            data[9] = 3;
            int ret = simAddDrawingObjectItem(handle, &data[0]);
            if(ret != 1)
            {
                // TODO: set error
                break;
            }
        }

        out->handle = handle;
    }

    void destroyGraph(destroyGraph_in *in, destroyGraph_out *out)
    {
        GraphContainer *g = graphContainerHandles.get(in->graphHandle);
        delete graphContainerHandles.remove(g);
    }

    VertexDesc closestVertexDescriptor(GraphContainer *g, octomap::point3d p)
    {
        double bestDist = -1;
        VertexDesc v;
        for(auto &kv : g->revmap)
        {
            double dist = p.distance(g->graph[kv.second].coord);
            if(dist < bestDist || bestDist < 0)
            {
                bestDist = dist;
                v = kv.second;
            }
        }
        return v;
    }

    void dijkstra(dijkstra_in *in, dijkstra_out *out)
    {
        GraphContainer *g = graphContainerHandles.get(in->graphHandle);
        octomap::point3d startPos(in->startPos[0], in->startPos[1], in->startPos[2]);
        octomap::point3d goalPos(in->goalPos[0], in->goalPos[1], in->goalPos[2]);
        VertexDesc start = closestVertexDescriptor(g, startPos),
                   goal = closestVertexDescriptor(g, goalPos);
        auto idmap = boost::get(boost::vertex_index, g->graph);
        std::vector<VertexDesc> predecessors(boost::num_vertices(g->graph));
        std::vector<double> distances(boost::num_vertices(g->graph));
        // we want all paths to goal (instead of all paths from start, like it usually is with dijkstra)
        dijkstra_shortest_paths(g->graph, goal,
            boost::weight_map(boost::get(&Edge::weight, g->graph))
            .distance_map(boost::make_iterator_property_map(distances.begin(), idmap))
            .predecessor_map(boost::make_iterator_property_map(predecessors.begin(), idmap))
        );
        std::vector<VertexDesc> path;
        VertexDesc current = start;
        while(1)
        {
            auto c = g->graph[current].coord;
            out->path.push_back(c.x());
            out->path.push_back(c.y());
            out->path.push_back(c.z());
            if(current == predecessors[current]) break;
            current = predecessors[current];
        }

        // distance map into octree
        if(in->createDistOctree)
        {
            int maxd = 0;
            for(VertexDesc v = 0; v < boost::num_vertices(g->graph); v++)
            {
                if(distances[v] > 2e9) continue;
                if(distances[v] > maxd) maxd = distances[v];
            }
            OcTree *distOctree = new OcTree(in->distOctreeResolution);
            out->distOctreeHandle = treeHandles.add(distOctree, in->_.scriptID);
            for(VertexDesc v = 0; v < boost::num_vertices(g->graph); v++)
            {
                if(distances[v] > 2e9) continue;
                auto c = g->graph[v].coord;
                float d = float(distances[v]) / float(maxd);
                distOctree->updateNode(c, d);
                distOctree->setNodeValue(c, d);
            }
        }
    }

    void getRoot(getRoot_in *in, getRoot_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        OcTreeNode *node = octree->getRoot();
        out->nodeHandle = Handle<OcTreeNode>::str(node);
    }

    void search(search_in *in, search_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        octomap::point3d coord = vectorToPoint(in->coord);
        OcTreeNode *node = octree->search(coord, in->depth);
        if(node)
            out->nodeHandle = Handle<OcTreeNode>::str(node);
    }

    void addValue(addValue_in *in, addValue_out *out)
    {
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        node->addValue(in->v);
    }

    void deleteChild(deleteChild_in *in, deleteChild_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        if(in->i < 0 || in->i >= 8)
            throw std::runtime_error("child index must be between 0 and 7");
        octree->deleteNodeChild(node, in->i);
    }

    void expandNode(expandNode_in *in, expandNode_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        octree->expandNode(node);
    }

    void setLogOdds(setLogOdds_in *in, setLogOdds_out *out)
    {
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        node->setLogOdds(in->v);
    }

    void setValue(setValue_in *in, setValue_out *out)
    {
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        node->setValue(in->v);
    }

    void setColor(setColor_in *in, setColor_out *out)
    {
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        if(in->r < 0 || in->r > 255 || in->g < 0 || in->g > 255 || in->b < 0 || in->b > 255)
            throw std::runtime_error("color component must be in 0..255 range");
        OcTreeNode::Color c(in->r, in->g, in->b);
        node->setColor(c);
    }

    void updateOccupancyChildren(updateOccupancyChildren_in *in, updateOccupancyChildren_out *out)
    {
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        node->updateOccupancyChildren();
    }

    void childExists(childExists_in *in, childExists_out *out)
    {
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        if(in->i < 0 || in->i >= 8)
            throw std::runtime_error("child index must be between 0 and 7");
        out->exists = node->childExists(in->i);
    }

    void collapsible(collapsible_in *in, collapsible_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        out->collapsible = octree->isNodeCollapsible(node);
    }

    void createChild(createChild_in *in, createChild_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        if(in->i < 0 || in->i >= 8)
            throw std::runtime_error("child index must be between 0 and 7");
        octree->createNodeChild(node, in->i);
    }

    void hasChildren(hasChildren_in *in, hasChildren_out *out)
    {
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        out->hasChildren = node->hasChildren();
    }

    void pruneNode(pruneNode_in *in, pruneNode_out *out)
    {
        OcTree *octree = treeHandles.get(in->octreeHandle);
        if(!octree)
            throw std::runtime_error("invalid OcTree handle");
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        octree->pruneNode(node);
    }

    void getValue(getValue_in *in, getValue_out *out)
    {
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        out->value = node->getValue();
    }

    void getColor(getColor_in *in, getColor_out *out)
    {
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        OcTreeNode::Color c = node->getColor();
        out->r = c.r;
        out->g = c.g;
        out->b = c.b;
    }

    void getLogOdds(getLogOdds_in *in, getLogOdds_out *out)
    {
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        out->value = node->getLogOdds();
    }

    void getMaxChildLogOdds(getMaxChildLogOdds_in *in, getMaxChildLogOdds_out *out)
    {
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        out->value = node->getMaxChildLogOdds();
    }

    void getMeanChildLogOdds(getMeanChildLogOdds_in *in, getMeanChildLogOdds_out *out)
    {
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        out->value = node->getMeanChildLogOdds();
    }

    void getOccupancy(getOccupancy_in *in, getOccupancy_out *out)
    {
        OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
        if(!node)
            throw std::runtime_error("invalid OcTreeNode handle");
        out->occupancy = node->getOccupancy();
    }

private:
    Handles<OcTree> treeHandles;
    Handles<GraphContainer> graphContainerHandles;
};

SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
#include "stubsPlusPlus.cpp"
