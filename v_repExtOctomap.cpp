// Copyright 2016 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// -------------------------------------------------------------------
// Authors:
// Federico Ferri <federico.ferri.it at gmail dot com>
// -------------------------------------------------------------------

#include "v_repExtOctomap.h"
#include "luaFunctionData.h"
#include "v_repLib.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

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
#include <octomap/math/Utils.h>

#include "stubs.h"

std::string luaTypeToString(simInt x)
{
    switch(x)
    {
    case sim_lua_arg_bool: return "sim_lua_arg_bool";
    case sim_lua_arg_int: return "sim_lua_arg_int";
    case sim_lua_arg_float: return "sim_lua_arg_float";
    case sim_lua_arg_double: return "sim_lua_arg_double";
    case sim_lua_arg_string: return "sim_lua_arg_string";
    case sim_lua_arg_charbuff: return "sim_lua_arg_charbuff";
    case sim_lua_arg_nil: return "sim_lua_arg_nil";
    case sim_lua_arg_table: return "sim_lua_arg_table";
    case sim_lua_arg_invalid: return "sim_lua_arg_invalid";
    }
    if(x & sim_lua_arg_nil)
        return luaTypeToString(x & ~sim_lua_arg_nil) + "|" + luaTypeToString(sim_lua_arg_nil);
    if(x & sim_lua_arg_table)
        return luaTypeToString(x & ~sim_lua_arg_table) + "|" + luaTypeToString(sim_lua_arg_table);
    return "???";
}

std::string luaCallbackToString(SLuaCallBack *c)
{
    std::stringstream ss;
    ss << "{inputArgsTypeAndSize: [";
    for(int i = 0; i < c->inputArgCount; i++)
    {
        ss << (i ? ", " : "") << luaTypeToString(c->inputArgTypeAndSize[2*i]);
        if(c->inputArgTypeAndSize[1+2*i]) ss << "_" << c->inputArgTypeAndSize[1+2*i];
    }
    ss << "], outputArgsTypeAndSize: [";
    for(int i = 0; i < c->outputArgCount; i++)
    {
        ss << (i ? ", " : "") << luaTypeToString(c->outputArgTypeAndSize[2*i]);
        if(c->outputArgTypeAndSize[1+2*i]) ss << "_" << c->outputArgTypeAndSize[1+2*i];
    }
    ss << "]}";
    return ss.str();
}

struct LuaCallbackFunction
{
    // name of the Lua function
    std::string function;
    // id of the V-REP script where the function is defined in
    simInt scriptId;
};

struct ObjectProxyHeader
{
    // internal handle of this object (used by the plugin):
    simInt handle;
    // objects created during simulation will be destroyed when simulation terminates:
    bool destroyAfterSimulationStop;
};

struct OctreeProxy
{
    ObjectProxyHeader header;
    // the octomap's OcTree itself
    octomap::OcTree *octree;

    OctreeProxy(octomap::OcTree *octree_) : octree(octree_) {}
    ~OctreeProxy() {if(octree) delete octree;}
};

std::map<simInt, OctreeProxy *> octrees;
simInt nextOctreeHandle = 1000;

// this function will be called at simulation end to destroy objects that
// were created during simulation, which otherwise would leak indefinitely:
template<typename T>
void destroyTransientObjects(std::map<simInt, T *>& c)
{
    std::vector<simInt> transientObjects;

    for(typename std::map<simInt, T *>::const_iterator it = c.begin(); it != c.end(); ++it)
    {
        if(it->second->header.destroyAfterSimulationStop)
            transientObjects.push_back(it->first);
    }

    for(size_t i = 0; i < transientObjects.size(); i++)
    {
        simInt key = transientObjects[i];
        T *t = c[key];
        c.erase(key);
        delete t;
    }
}

void destroyTransientObjects()
{
    destroyTransientObjects(octrees);
}

void create(SLuaCallBack *p, const char *cmd, create_in *in, create_out *out)
{
    OctreeProxy *o = new OctreeProxy(new octomap::OcTree(in->resolution));
    o->header.handle = nextOctreeHandle++;
    octrees[o->header.handle] = o;
    out->octreeHandle = o->header.handle;
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

octomap::point3d snapCoord(octomap::OcTree *tree, int depth, octomap::point3d coord)
{
    octomap::OcTreeKey key = tree->coordToKey(coord, depth);
    octomap::point3d coord1 = tree->keyToCoord(key, depth);
    return coord1;
}

void measureOccupancy(octomap::OcTree *tree, int depth, octomap::point3d coord, const octomap::point3d& boundsMin, const octomap::point3d& boundsMax, std::map<int,simInt>& proximitySensors)
{
    double nodeSize = tree->getNodeSize(depth);

    // check if the voxel being measured is completely out of bounds, and in that case skip:
    if(coord.x() + 0.5 * nodeSize < boundsMin.x() || coord.x() - 0.5 * nodeSize > boundsMax.x()
            || coord.y() + 0.5 * nodeSize < boundsMin.y() || coord.y() - 0.5 * nodeSize > boundsMax.y()
            || coord.z() + 0.5 * nodeSize < boundsMin.z() || coord.z() - 0.5 * nodeSize > boundsMax.z())
        return;

    if(proximitySensors.find(depth) == proximitySensors.end())
    {
        proximitySensors[depth] = createProximitySensor(nodeSize);
    }
    simInt sens = proximitySensors[depth];

    simFloat p[4];
    p[0] = coord.x();
    p[1] = coord.y();
    p[2] = coord.z() - nodeSize * 0.5; // proximity sensor's origin is in its XY plane!
    simSetObjectPosition(sens, -1, &p[0]);
    simInt r = simCheckProximitySensor(sens, sim_handle_all, &p[0]);
    
    if(r == 1) // some obstacle detected
    {
        if(depth == tree->getTreeDepth()) // maximum depth -> single update
        {
            tree->updateNode(coord, true);
        }
        else // recursive update
        {
            int depth1 = depth + 1;
            double nodeSize1 = tree->getNodeSize(depth1);
            double off1 = 0.5 * (nodeSize - nodeSize1);
            measureOccupancy(tree, depth1, coord + octomap::point3d(-off1, -off1, -off1), boundsMin, boundsMax, proximitySensors);
            measureOccupancy(tree, depth1, coord + octomap::point3d(+off1, -off1, -off1), boundsMin, boundsMax, proximitySensors);
            measureOccupancy(tree, depth1, coord + octomap::point3d(-off1, +off1, -off1), boundsMin, boundsMax, proximitySensors);
            measureOccupancy(tree, depth1, coord + octomap::point3d(+off1, +off1, -off1), boundsMin, boundsMax, proximitySensors);
            measureOccupancy(tree, depth1, coord + octomap::point3d(-off1, -off1, +off1), boundsMin, boundsMax, proximitySensors);
            measureOccupancy(tree, depth1, coord + octomap::point3d(+off1, -off1, +off1), boundsMin, boundsMax, proximitySensors);
            measureOccupancy(tree, depth1, coord + octomap::point3d(-off1, +off1, +off1), boundsMin, boundsMax, proximitySensors);
            measureOccupancy(tree, depth1, coord + octomap::point3d(+off1, +off1, +off1), boundsMin, boundsMax, proximitySensors);
        }
    }
    else if(r == 0) // no obstacle detected -> mark area as empty
    {
        int maxDepth = tree->getTreeDepth();
        octomap::point3d boundsMinSnap = snapCoord(tree, maxDepth, boundsMin);
        octomap::point3d boundsMaxSnap = snapCoord(tree, maxDepth, boundsMax);
        double res = tree->getResolution();
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
                    tree->updateNode(x, y, z, false);
                }
            }
        }
    }
}

void createFromScene(SLuaCallBack *p, const char *cmd, createFromScene_in *in, createFromScene_out *out)
{
    if(in->boundsMin.size() != 3 || in->boundsMax.size() != 3)
    {
        simSetLastError(cmd, "bounds must have 3 elements");
        return;
    }

    for(int i = 0; i < 3; i++)
    {
        if(in->boundsMin[i] >= in->boundsMax[i])
        {
            simSetLastError(cmd, "bounds min must be strictly lower than max");
            return;
        }
    }

    octomap::OcTree *tree = new octomap::OcTree(in->resolution);

#ifdef FAST_OCCUPANCY_MEASUREMENT
    // snap bounds to voxel coords at minimum depth:
    int count = 0;
    int depth = 1;
    octomap::point3d boundsMin(in->boundsMin[0], in->boundsMin[1], in->boundsMin[2]);
    octomap::point3d boundsMax(in->boundsMax[0], in->boundsMax[1], in->boundsMax[2]);
    octomap::point3d boundsMinSnap = snapCoord(tree, depth, boundsMin);
    octomap::point3d boundsMaxSnap = snapCoord(tree, depth, boundsMax);
    double nodeSize = tree->getNodeSize(depth);

    // V-REP's proximity sensors, by depth:
    std::map<int,simInt> proximitySensors;

    for(double z = boundsMinSnap.z(); z <= boundsMaxSnap.z(); z += nodeSize)
    {
        for(double y = boundsMinSnap.y(); y <= boundsMaxSnap.y(); y += nodeSize)
        {
            for(double x = boundsMinSnap.x(); x <= boundsMaxSnap.x(); x += nodeSize)
            {
                measureOccupancy(tree, depth, octomap::point3d(x, y, z), boundsMin, boundsMax, proximitySensors);
            }
        }
    }

    for(std::map<int,simInt>::iterator it = proximitySensors.begin(); it != proximitySensors.end(); ++it)
        simRemoveObject(it->second);

#else
    simInt sens = createProximitySensor(in->resolution);

    for(float z = in->boundsMin[2]; z <= in->boundsMax[2]; z += in->resolution)
    {
        for(float y = in->boundsMin[1]; y <= in->boundsMax[1]; y += in->resolution)
        {
            for(float x = in->boundsMin[0]; x <= in->boundsMax[0]; x += in->resolution)
            {
                simFloat p[4];
                p[0] = x;
                p[1] = y;
                p[2] = z - 0.5 * in->resolution;
                simSetObjectPosition(sens, -1, &p[0]);
                simInt r = simCheckProximitySensor(sens, sim_handle_all, &p[0]);
                bool occ = r == 1;
                tree->updateNode(x, y, z, occ);
            }
        }
    }

    simRemoveObject(sens);
#endif

    OctreeProxy *o = new OctreeProxy(tree);
    o->header.handle = nextOctreeHandle++;
    octrees[o->header.handle] = o;
    out->octreeHandle = o->header.handle;
}

OctreeProxy * getOctreeOrSetError(const char *cmd, simInt octreeHandle)
{
    if(octrees.find(octreeHandle) == octrees.end())
    {
        simSetLastError(cmd, "Invalid OcTree handle.");
        return NULL;
    }

    return octrees[octreeHandle];
}

void destroy(SLuaCallBack *p, const char *cmd, destroy_in *in, destroy_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    octrees.erase(in->octreeHandle);
    delete o;
    out->result = 1;
}

octomap::point3d vectorToPoint(std::vector<float> v)
{
    return octomap::point3d(v[0], v[1], v[2]);
}

octomap::OcTreeKey vectorToKey(std::vector<int> v)
{
    octomap::OcTreeKey key;
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

std::vector<int> keyToVector(octomap::OcTreeKey k)
{
    std::vector<int> v;
    v.push_back(k[0]);
    v.push_back(k[1]);
    v.push_back(k[2]);
    return v;
}

void clear(SLuaCallBack *p, const char *cmd, clear_in *in, clear_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    o->octree->clear();
    out->result = 1;
}

void coordToKey(SLuaCallBack *p, const char *cmd, coordToKey_in *in, coordToKey_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    octomap::point3d coord = vectorToPoint(in->coord);
    octomap::OcTreeKey key = o->octree->coordToKey(coord);
    out->key = keyToVector(key);
}

void keyToCoord(SLuaCallBack *p, const char *cmd, keyToCoord_in *in, keyToCoord_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    octomap::OcTreeKey key = vectorToKey(in->key);
    octomap::point3d coord = o->octree->keyToCoord(key);
    out->coord = pointToVector(coord);
}

void deleteNode(SLuaCallBack *p, const char *cmd, deleteNode_in *in, deleteNode_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    octomap::point3d coord = vectorToPoint(in->coord);
    o->octree->deleteNode(coord, in->depth);
    out->result = 1;
}

void deleteNodeWithKey(SLuaCallBack *p, const char *cmd, deleteNodeWithKey_in *in, deleteNodeWithKey_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    octomap::OcTreeKey key = vectorToKey(in->key);
    o->octree->deleteNode(key, in->depth);
    out->result = 1;
}

void getMetricBounds(SLuaCallBack *p, const char *cmd, getMetricBounds_in *in, getMetricBounds_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    double minx, miny, minz, maxx, maxy, maxz, sizex, sizey, sizez;
    o->octree->getMetricMin(minx, miny, minz);
    o->octree->getMetricMax(maxx, maxy, maxz);
    o->octree->getMetricSize(sizex, sizey, sizez);
    out->boundsMin = boost::assign::list_of(minx)(miny)(minz);
    out->boundsMax = boost::assign::list_of(maxx)(maxy)(maxz);
    out->size = boost::assign::list_of(sizex)(sizey)(sizez);
}

void getNodeSize(SLuaCallBack *p, const char *cmd, getNodeSize_in *in, getNodeSize_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    out->size = o->octree->getNodeSize(in->depth);
}

void getNumLeafNodes(SLuaCallBack *p, const char *cmd, getNumLeafNodes_in *in, getNumLeafNodes_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    out->n = o->octree->getNumLeafNodes();
}

void getSize(SLuaCallBack *p, const char *cmd, getSize_in *in, getSize_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    out->size = o->octree->size();
}

void getVolume(SLuaCallBack *p, const char *cmd, getVolume_in *in, getVolume_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    out->volume = o->octree->volume();
}

void getResolution(SLuaCallBack *p, const char *cmd, getResolution_in *in, getResolution_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    out->res = o->octree->getResolution();
}

void getTreeDepth(SLuaCallBack *p, const char *cmd, getTreeDepth_in *in, getTreeDepth_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    out->depth = o->octree->getTreeDepth();
}

void getTreeType(SLuaCallBack *p, const char *cmd, getTreeType_in *in, getTreeType_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    out->treeType = o->octree->getTreeType();
}

void prune(SLuaCallBack *p, const char *cmd, prune_in *in, prune_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    o->octree->prune();
    out->result = 1;
}

void updateNode(SLuaCallBack *p, const char *cmd, updateNode_in *in, updateNode_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    octomap::point3d coord = vectorToPoint(in->coord);
    switch(in->mode)
    {
    case sim_octomap_update_log_odds:
        o->octree->updateNode(coord, in->log_odds);
        out->result = 1;
        break;
    case sim_octomap_update_occupancy:
        o->octree->updateNode(coord, in->occupancy);
        out->result = 1;
        break;
    default:
        simSetLastError(cmd, "invalid update mode");
    }
}

void updateNodeWithKey(SLuaCallBack *p, const char *cmd, updateNodeWithKey_in *in, updateNodeWithKey_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    octomap::OcTreeKey key = vectorToKey(in->key);
    switch(in->mode)
    {
    case sim_octomap_update_log_odds:
        o->octree->updateNode(key, in->log_odds);
        out->result = 1;
        break;
    case sim_octomap_update_occupancy:
        o->octree->updateNode(key, in->occupancy);
        out->result = 1;
        break;
    default:
        simSetLastError(cmd, "invalid update mode");
    }
}

void insertPointCloud(SLuaCallBack *p, const char *cmd, insertPointCloud_in *in, insertPointCloud_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;

    if(!in->points.size()) return;

    if(in->origin.size() != 3)
    {
        simSetLastError(cmd, "origin must be a table of size 3");
        return;
    }

    if(in->points.size() % 3)
    {
        simSetLastError(cmd, "points size is not a multiple of 3");
        return;
    }

    simSetLastError(cmd, "not implemented yet");
}

void castRay(SLuaCallBack *p, const char *cmd, castRay_in *in, castRay_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;

    if(in->origin.size() != 3)
    {
        simSetLastError(cmd, "origin must be a table of size 3");
        return;
    }

    if(in->direction.size() != 3)
    {
        simSetLastError(cmd, "direction must be a table of size 3");
        return;
    }

    simSetLastError(cmd, "not implemented yet");
}

void write(SLuaCallBack *p, const char *cmd, write_in *in, write_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    o->octree->write(in->filename);
    out->result = 1;
}

void writeBinary(SLuaCallBack *p, const char *cmd, writeBinary_in *in, writeBinary_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    o->octree->writeBinary(in->filename);
    out->result = 1;
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

void addDrawingObject(SLuaCallBack *p, const char *cmd, addDrawingObject_in *in, addDrawingObject_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;

    if(in->voxelColor == sim_octomap_voxelcolor_flat && in->flatColor.size() != 3)
    {
        simSetLastError(cmd, "flatColor must have 3 elements");
        return;
    }

    double minX, minY, minZ, sizeX, sizeY, sizeZ;
    o->octree->getMetricMin(minX, minY, minZ);
    o->octree->getMetricSize(sizeX, sizeY, sizeZ);

    simInt handle = simAddDrawingObject(sim_drawing_cubepoints + sim_drawing_itemcolors + sim_drawing_itemsizes, 0, 0.0, -1, 1000000, NULL, NULL, NULL, NULL);
    simFloat data[10];

    octomap::OcTree::leaf_iterator begin = o->octree->begin(in->depth), end = o->octree->end();
    for(octomap::OcTree::leaf_iterator it = begin; it != end; ++it)
    {
        if(!o->octree->isNodeOccupied(*it) && in->skipFree) continue;
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

void isOccupied(SLuaCallBack *p, const char *cmd, isOccupied_in *in, isOccupied_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    octomap::point3d coord = vectorToPoint(in->coord);
    octomap::OcTreeNode *node = o->octree->search(coord, in->depth);
    if(!node)
    {
        out->result = -1;
    }
    else
    {
        out->result = o->octree->isNodeOccupied(node) ? 1 : 0;
    }
}

void isOccupiedKey(SLuaCallBack *p, const char *cmd, isOccupiedKey_in *in, isOccupiedKey_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;
    octomap::OcTreeKey key = vectorToKey(in->key);
    octomap::OcTreeNode *node = o->octree->search(key, in->depth);
    if(!node)
    {
        out->result = -1;
    }
    else
    {
        out->result = o->octree->isNodeOccupied(node) ? 1 : 0;
    }
}

void f(SLuaCallBack *p, const char *cmd, f_in *in, f_out *out)
{
    OctreeProxy *o = getOctreeOrSetError(cmd, in->octreeHandle);
    if(!o) return;

    OctreeProxy *r = new OctreeProxy(new octomap::OcTree(o->octree->getResolution()));
    r->header.handle = nextOctreeHandle++;
    octrees[r->header.handle] = r;
    out->octreeHandle = r->header.handle;

    octomap::OcTree::leaf_iterator begin = o->octree->begin(), end = o->octree->end();
    for(octomap::OcTree::leaf_iterator it = begin; it != end; ++it)
    {
        if(!o->octree->isNodeOccupied(*it)) continue;

        octomap::OcTreeKey key = it.getKey();
        key[2]++;
        octomap::OcTreeNode *node = o->octree->search(key);
        if(node && !o->octree->isNodeOccupied(node))
        {
            r->octree->updateNode(key, true);
        }
    }
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

    registerLuaStuff();

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
        destroyTransientObjects();
    }

    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved); // restore previous settings
    return(retVal);
}

