#include "v_repExtOctomap.h"
#include "v_repLib.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

#include <boost/format.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/predicate.hpp>

#ifdef _WIN32
    #ifdef QT_COMPIL
        #include <direct.h>
    #else
        #include <shlwapi.h>
        #pragma comment(lib, "Shlwapi.lib")
    #endif
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
    #include <unistd.h>
#define _stricmp strcasecmp
#endif /* __linux || __APPLE__ */

#define CONCAT(x, y, z) x y z
#define strConCat(x, y, z)    CONCAT(x, y, z)

#define PLUGIN_VERSION 3 // 3 since V3.3.0, 2 since V3.3.0Beta.

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/math/Utils.h>

#include "stubs.h"

#define PROXIMITY_SENSOR_INFLATE 0.001

// handle: a tool for pointer <--> string conversion
template<typename T>
struct Handle
{
    static std::string str(const T *t)
    {
        static boost::format fmt("%s:%lld:%d");
        return (fmt % tag() % reinterpret_cast<long long int>(t) % crc_ptr(t)).str();
    }

    static T * obj(std::string h)
    {
        boost::cmatch m;
        static boost::regex re("([^:]+):([^:]+):([^:]+)");
        if(boost::regex_match(h.c_str(), m, re) && m[1] == tag())
        {
            T *t = reinterpret_cast<T*>(boost::lexical_cast<long long int>(m[2]));
            int crc = boost::lexical_cast<int>(m[3]);
            if(crc == crc_ptr(t)) return t;
        }
        return nullptr;
    }

private:
    static std::string tag()
    {
        return "ptr";
    }

    static int crc_ptr(const T *t)
    {
        auto x = reinterpret_cast<long long int>(t);
        x = x ^ (x >> 32);
        x = x ^ (x >> 16);
        x = x ^ (x >> 8);
        x = x ^ (x >> 4);
        x = x & 0x000000000000000F;
        x = x ^ 0x0000000000000008;
        return int(x);
    }
};

using OcTree = octomap::ColorOcTree;
using OcTreeKey = octomap::OcTreeKey;
using OcTreeNode = octomap::ColorOcTreeNode;

template<> std::string Handle<OcTree>::tag() { return "octomap.OcTree"; }
template<> std::string Handle<OcTreeNode>::tag() { return "octomap.OcTreeNode"; }

void create(SScriptCallBack *p, const char *cmd, create_in *in, create_out *out)
{
    OcTree *octree = new OcTree(in->resolution);
    out->octreeHandle = Handle<OcTree>::str(octree);
}

simInt createProximitySensor(float size)
{
    simInt options = 0 // sensor options:
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
    simInt intParams[] = {
            0,     // face count (volume description)
            0,     // face count far (volume description)
            0,     // subdivisions (volume description)
            0,     // subdivisions far (volume description)
            0,     // randomized detection, sample count per reading
            0,     // randomized detection, individual ray detection count for triggering
            0,     // reserved. Set to 0
            0      // reserved. Set to 0
    };
    simFloat floatParams[] = {
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

void measureOccupancy(OcTree *octree, int depth, octomap::point3d coord, const octomap::point3d& boundsMin, const octomap::point3d& boundsMax, std::map<int,simInt>& proximitySensors)
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
    simInt sens = proximitySensors[depth];

    simFloat p[4];
    p[0] = coord.x();
    p[1] = coord.y();
    p[2] = coord.z() - nodeSize * 0.5 - PROXIMITY_SENSOR_INFLATE * 0.5; // proximity sensor's origin is in its XY plane!
    simSetObjectPosition(sens, -1, &p[0]);
    simInt r = simCheckProximitySensor(sens, sim_handle_all, &p[0]);
    
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

void createFromScene(SScriptCallBack *p, const char *cmd, createFromScene_in *in, createFromScene_out *out)
{
    if(in->boundsMin.size() != 3 || in->boundsMax.size() != 3)
        throw std::string("bounds must have 3 elements");

    for(int i = 0; i < 3; i++)
        if(in->boundsMin[i] >= in->boundsMax[i])
            throw std::string("bounds min must be strictly lower than max");

    OcTree *octree = new OcTree(in->resolution);

#ifndef DISABLE_FAST_OCCUPANCY_MEASUREMENT
    // snap bounds to voxel coords at minimum depth:
    int depth = 6;
    octomap::point3d boundsMin(in->boundsMin[0], in->boundsMin[1], in->boundsMin[2]);
    octomap::point3d boundsMax(in->boundsMax[0], in->boundsMax[1], in->boundsMax[2]);
    octomap::point3d boundsMinSnap = snapCoord(octree, depth, boundsMin);
    octomap::point3d boundsMaxSnap = snapCoord(octree, depth, boundsMax);
    double nodeSize = octree->getNodeSize(depth);

    // V-REP's proximity sensors, by depth:
    std::map<int,simInt> proximitySensors;

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

    for(std::map<int,simInt>::iterator it = proximitySensors.begin(); it != proximitySensors.end(); ++it)
        simRemoveObject(it->second);

#else
    simInt sens = createProximitySensor(in->resolution + PROXIMITY_SENSOR_INFLATE);

    for(float z = in->boundsMin[2]; z <= in->boundsMax[2]; z += in->resolution)
    {
        for(float y = in->boundsMin[1]; y <= in->boundsMax[1]; y += in->resolution)
        {
            for(float x = in->boundsMin[0]; x <= in->boundsMax[0]; x += in->resolution)
            {
                simFloat p[4];
                p[0] = x;
                p[1] = y;
                p[2] = z - 0.5 * in->resolution - 0.5 * PROXIMITY_SENSOR_INFLATE;
                simSetObjectPosition(sens, -1, &p[0]);
                simInt r = simCheckProximitySensor(sens, sim_handle_all, &p[0]);
                bool occ = r == 1;
                octree->updateNode(x, y, z, occ);
            }
        }
    }

    simRemoveObject(sens);
#endif

    out->octreeHandle = Handle<OcTree>::str(octree);
}

void destroy(SScriptCallBack *p, const char *cmd, destroy_in *in, destroy_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
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

void clear(SScriptCallBack *p, const char *cmd, clear_in *in, clear_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    octree->clear();
}

void coordToKey(SScriptCallBack *p, const char *cmd, coordToKey_in *in, coordToKey_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    octomap::point3d coord = vectorToPoint(in->coord);
    OcTreeKey key = octree->coordToKey(coord);
    out->key = keyToVector(key);
}

void keyToCoord(SScriptCallBack *p, const char *cmd, keyToCoord_in *in, keyToCoord_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    OcTreeKey key = vectorToKey(in->key);
    octomap::point3d coord = octree->keyToCoord(key);
    out->coord = pointToVector(coord);
}

void deleteNode(SScriptCallBack *p, const char *cmd, deleteNode_in *in, deleteNode_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    octomap::point3d coord = vectorToPoint(in->coord);
    octree->deleteNode(coord, in->depth);
}

void deleteNodeWithKey(SScriptCallBack *p, const char *cmd, deleteNodeWithKey_in *in, deleteNodeWithKey_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    OcTreeKey key = vectorToKey(in->key);
    octree->deleteNode(key, in->depth);
}

void getMetricBounds(SScriptCallBack *p, const char *cmd, getMetricBounds_in *in, getMetricBounds_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    double minx, miny, minz, maxx, maxy, maxz, sizex, sizey, sizez;
    octree->getMetricMin(minx, miny, minz);
    octree->getMetricMax(maxx, maxy, maxz);
    octree->getMetricSize(sizex, sizey, sizez);
    out->boundsMin = {float(minx), float(miny), float(minz)};
    out->boundsMax = {float(maxx), float(maxy), float(maxz)};
    out->size = {float(sizex), float(sizey), float(sizez)};
}

void getNodeSize(SScriptCallBack *p, const char *cmd, getNodeSize_in *in, getNodeSize_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    out->size = octree->getNodeSize(in->depth);
}

void getNumLeafNodes(SScriptCallBack *p, const char *cmd, getNumLeafNodes_in *in, getNumLeafNodes_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    out->n = octree->getNumLeafNodes();
}

void getSize(SScriptCallBack *p, const char *cmd, getSize_in *in, getSize_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    out->size = octree->size();
}

void getVolume(SScriptCallBack *p, const char *cmd, getVolume_in *in, getVolume_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    out->volume = octree->volume();
}

void getResolution(SScriptCallBack *p, const char *cmd, getResolution_in *in, getResolution_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    out->res = octree->getResolution();
}

void getTreeDepth(SScriptCallBack *p, const char *cmd, getTreeDepth_in *in, getTreeDepth_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    out->depth = octree->getTreeDepth();
}

void getTreeType(SScriptCallBack *p, const char *cmd, getTreeType_in *in, getTreeType_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    out->treeType = octree->getTreeType();
}

void prune(SScriptCallBack *p, const char *cmd, prune_in *in, prune_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    octree->prune();
}

void updateNode(SScriptCallBack *p, const char *cmd, updateNode_in *in, updateNode_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
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
        throw std::string("invalid update mode");
    }
}

void updateNodeWithKey(SScriptCallBack *p, const char *cmd, updateNodeWithKey_in *in, updateNodeWithKey_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
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
        throw std::string("invalid update mode");
    }
}

void insertPointCloud(SScriptCallBack *p, const char *cmd, insertPointCloud_in *in, insertPointCloud_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");

    if(!in->points.size()) return;

    if(in->origin.size() != 3)
        throw std::string("origin must be a table of size 3");

    if(in->points.size() % 3)
        throw std::string("points size is not a multiple of 3");

    int n = in->points.size() / 3;
    octomap::Pointcloud cloud;
    for(int i = 0; i < n; i++)
        cloud.push_back(in->points[i*3+0], in->points[i*3+1], in->points[i*3+2]);
    octomap::point3d origin = vectorToPoint(in->origin);
    octomap::pose6d frame_origin(0, 0, 0, 0, 0, 0);
    octree->insertPointCloud(cloud, origin, frame_origin, in->maxRange);
}

void castRay(SScriptCallBack *p, const char *cmd, castRay_in *in, castRay_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");

    if(in->origin.size() != 3)
        throw std::string("origin must be a table of size 3");

    if(in->direction.size() != 3)
        throw std::string("direction must be a table of size 3");

    octomap::point3d origin = vectorToPoint(in->origin);
    octomap::point3d direction = vectorToPoint(in->direction);
    octomap::point3d end;
    out->hit = octree->castRay(origin, direction, end, in->ignoreUnknownCells, in->maxRange);
    out->end = pointToVector(end);
}

void write(SScriptCallBack *p, const char *cmd, write_in *in, write_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    octree->write(in->filename);
}

void writeBinary(SScriptCallBack *p, const char *cmd, writeBinary_in *in, writeBinary_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    octree->writeBinary(in->filename);
}

void valueColor(float value, float& r, float& g, float& b)
{
    if(value < 0.0) value = 0.0;
    if(value > 1.0) value = 1.0;
    value = 1.0 - value;
    int fi = int(value * 6.) % 6;
    float ff = value * 6. - fi;
    if(ff < 0.0) ff += 1.0;
    switch(fi) {
    case 0: r = 1.0;      g = ff;       b = 0.0;      break;
    case 1: r = 1.0 - ff; g = 1.0;      b = 0.0;      break;
    case 2: r = 0.0;      g = 1.0;      b = ff;       break;
    case 3: r = 0.0;      g = 1.0 - ff; b = 1.0;      break;
    case 4: r = ff;       g = 0.0;      b = 1.0;      break;
    case 5: r = 1.0;      g = 0.0;      b = 1.0 - ff; break;
    }
}

void addDrawingObject(SScriptCallBack *p, const char *cmd, addDrawingObject_in *in, addDrawingObject_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");

    if(in->voxelColor == sim_octomap_voxelcolor_flat && in->flatColor.size() != 3)
        throw std::string("flatColor must have 3 elements");

    double minX, minY, minZ, sizeX, sizeY, sizeZ;
    octree->getMetricMin(minX, minY, minZ);
    octree->getMetricSize(sizeX, sizeY, sizeZ);

    simInt handle = simAddDrawingObject(sim_drawing_cubepoints + sim_drawing_itemcolors + sim_drawing_itemsizes, 0, 0.0, -1, 1000000, NULL, NULL, NULL, NULL);
    simFloat data[10];

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

        simInt ret = simAddDrawingObjectItem(handle, &data[0]);
        if(ret != 1)
        {
            // TODO: set error
            break;
        }
    }

    out->handle = handle;
}

void isOccupied(SScriptCallBack *p, const char *cmd, isOccupied_in *in, isOccupied_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
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

void isOccupiedKey(SScriptCallBack *p, const char *cmd, isOccupiedKey_in *in, isOccupiedKey_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
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

void f(SScriptCallBack *p, const char *cmd, f_in *in, f_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    octree->expand();

    OcTree *gnd = new OcTree(octree->getResolution());
    out->gndOctreeHandle = Handle<OcTree>::str(gnd);

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
    octree->prune();
}

void getRoot(SScriptCallBack *p, const char *cmd, getRoot_in *in, getRoot_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    OcTreeNode *node = octree->getRoot();
    out->nodeHandle = Handle<OcTreeNode>::str(node);
}

void search(SScriptCallBack *p, const char *cmd, search_in *in, search_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    octomap::point3d coord = vectorToPoint(in->coord);
    OcTreeNode *node = octree->search(coord, in->depth);
    out->nodeHandle = Handle<OcTreeNode>::str(node);
}

void addValue(SScriptCallBack *p, const char *cmd, addValue_in *in, addValue_out *out)
{
    OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
    if(!node)
        throw std::string("invalid OcTreeNode handle");
    node->addValue(in->v);
}

void deleteChild(SScriptCallBack *p, const char *cmd, deleteChild_in *in, deleteChild_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
    if(!node)
        throw std::string("invalid OcTreeNode handle");
    if(in->i < 0 || in->i >= 8)
        throw std::string("child index must be between 0 and 7");
    octree->deleteNodeChild(node, in->i);
}

void expandNode(SScriptCallBack *p, const char *cmd, expandNode_in *in, expandNode_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
    if(!node)
        throw std::string("invalid OcTreeNode handle");
    octree->expandNode(node);
}

void setLogOdds(SScriptCallBack *p, const char *cmd, setLogOdds_in *in, setLogOdds_out *out)
{
    OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
    if(!node)
        throw std::string("invalid OcTreeNode handle");
    node->setLogOdds(in->v);
}

void setValue(SScriptCallBack *p, const char *cmd, setValue_in *in, setValue_out *out)
{
    OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
    if(!node)
        throw std::string("invalid OcTreeNode handle");
    node->setValue(in->v);
}

void updateOccupancyChildren(SScriptCallBack *p, const char *cmd, updateOccupancyChildren_in *in, updateOccupancyChildren_out *out)
{
    OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
    if(!node)
        throw std::string("invalid OcTreeNode handle");
    node->updateOccupancyChildren();
}

void childExists(SScriptCallBack *p, const char *cmd, childExists_in *in, childExists_out *out)
{
    OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
    if(!node)
        throw std::string("invalid OcTreeNode handle");
    if(in->i < 0 || in->i >= 8)
        throw std::string("child index must be between 0 and 7");
    out->exists = node->childExists(in->i);
}

void collapsible(SScriptCallBack *p, const char *cmd, collapsible_in *in, collapsible_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
    if(!node)
        throw std::string("invalid OcTreeNode handle");
    out->collapsible = octree->isNodeCollapsible(node);
}

void createChild(SScriptCallBack *p, const char *cmd, createChild_in *in, createChild_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
    if(!node)
        throw std::string("invalid OcTreeNode handle");
    if(in->i < 0 || in->i >= 8)
        throw std::string("child index must be between 0 and 7");
    octree->createNodeChild(node, in->i);
}

void hasChildren(SScriptCallBack *p, const char *cmd, hasChildren_in *in, hasChildren_out *out)
{
    OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
    if(!node)
        throw std::string("invalid OcTreeNode handle");
    out->hasChildren = node->hasChildren();
}

void pruneNode(SScriptCallBack *p, const char *cmd, pruneNode_in *in, pruneNode_out *out)
{
    OcTree *octree = Handle<OcTree>::obj(in->octreeHandle);
    if(!octree)
        throw std::string("invalid OcTree handle");
    OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
    if(!node)
        throw std::string("invalid OcTreeNode handle");
    octree->pruneNode(node);
}

void getValue(SScriptCallBack *p, const char *cmd, getValue_in *in, getValue_out *out)
{
    OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
    if(!node)
        throw std::string("invalid OcTreeNode handle");
    out->value = node->getValue();
}

void getLogOdds(SScriptCallBack *p, const char *cmd, getLogOdds_in *in, getLogOdds_out *out)
{
    OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
    if(!node)
        throw std::string("invalid OcTreeNode handle");
    out->value = node->getLogOdds();
}

void getMaxChildLogOdds(SScriptCallBack *p, const char *cmd, getMaxChildLogOdds_in *in, getMaxChildLogOdds_out *out)
{
    OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
    if(!node)
        throw std::string("invalid OcTreeNode handle");
    out->value = node->getMaxChildLogOdds();
}

void getMeanChildLogOdds(SScriptCallBack *p, const char *cmd, getMeanChildLogOdds_in *in, getMeanChildLogOdds_out *out)
{
    OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
    if(!node)
        throw std::string("invalid OcTreeNode handle");
    out->value = node->getMeanChildLogOdds();
}

void getOccupancy(SScriptCallBack *p, const char *cmd, getOccupancy_in *in, getOccupancy_out *out)
{
    OcTreeNode *node = Handle<OcTreeNode>::obj(in->nodeHandle);
    if(!node)
        throw std::string("invalid OcTreeNode handle");
    out->occupancy = node->getOccupancy();
}

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer, int reservedInt)
{
    char curDirAndFile[1024];
#ifdef _WIN32
    #ifdef QT_COMPIL
        _getcwd(curDirAndFile, sizeof(curDirAndFile));
    #else
        GetModuleFileName(NULL, curDirAndFile, 1023);
        PathRemoveFileSpec(curDirAndFile);
    #endif
#elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

    std::string currentDirAndPath(curDirAndFile);
    std::string temp(currentDirAndPath);
#ifdef _WIN32
    temp+="\\v_rep.dll";
#elif defined (__linux)
    temp+="/libv_rep.so";
#elif defined (__APPLE__)
    temp+="/libv_rep.dylib";
#endif /* __linux || __APPLE__ */
    vrepLib = loadVrepLibrary(temp.c_str());
    if (vrepLib == NULL)
    {
        std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'Octomap' plugin.\n";
        return(0);
    }
    if (getVrepProcAddresses(vrepLib)==0)
    {
        std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'Octomap' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return(0);
    }

    int vrepVer;
    simGetIntegerParameter(sim_intparam_program_version, &vrepVer);
    if (vrepVer < 30203) // if V-REP version is smaller than 3.02.03
    {
        std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'Octomap' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return(0);
    }

    if(!registerScriptStuff())
    {
        std::cout << "Initialization failed.\n";
        unloadVrepLibrary(vrepLib);
        return(0);
    }

    return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

VREP_DLLEXPORT void v_repEnd()
{
    unloadVrepLibrary(vrepLib); // release the library
}

VREP_DLLEXPORT void* v_repMessage(int message, int* auxiliaryData, void* customData, int* replyData)
{
    // Keep following 5 lines at the beginning and unchanged:
    static bool refreshDlgFlag = true;
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);
    void* retVal=NULL;

    if (message == sim_message_eventcallback_simulationended)
    { // Simulation just ended
    }

    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved); // restore previous settings
    return(retVal);
}

