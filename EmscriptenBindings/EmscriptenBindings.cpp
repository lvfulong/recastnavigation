
#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <emscripten/html5.h>

#include <stdlib.h>
#include <stdio.h>

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "DetourCrowd.h"
#include "DetourNavMeshBuilder.h"
#include "Recast.h"
#include "NavMeshObstacls.h"

using namespace emscripten;

static float frand()
{
    return ((float)(rand() & 0xffff) / (float)0xffff);
}
template <typename T>
val CreateArray(const T *data, int maxLength)
{
    std::vector<T> lists = std::vector<T>(data, data + maxLength);
    return val::array(lists);
};

template <typename T>
val CreateArrayView(const T *data, int maxLength)
{
    return val(typed_memory_view(maxLength, data));
};

template <typename T>
T *GetArray(val data, intptr_t *sizeLength)
{
    const std::vector<T> pathData = convertJSArrayToNumberVector<T>(data);
    const intptr_t size = pathData.size();
    T *datas = (T *)malloc(size * sizeof(T));
    for (int i = 0; i < size; i++)
    {
        datas[i] = pathData.data()[i];
    }
    *sizeLength = size;
    return datas;
};

template <typename T>
void writeToArray(val data, T *path, intptr_t sizeLength)
{
    for (intptr_t i = 0; i < sizeLength; i++)
    {
        data.set(i, path[i]);
    }
};

void writeToJsObject(val value, const float *data)
{
    value.set("x", val(data[0]));
    value.set("y", val(data[1]));
    value.set("z", val(data[2]));
}

class dtRefPointData
{
public:
    dtPolyRef dtpolyRef;
    float data[3];
    float getx()
    {
        return data[0];
    }
    float gety()
    {
        return data[1];
    }
    float getz()
    {
        return data[2];
    }

    void setx(float value)
    {
        data[0] = value;
    }

    void sety(float value)
    {
        data[1] = value;
    }

    void setz(float value)
    {
        data[2] = value;
    }

    dtPolyRef getployRef()
    {
        return dtpolyRef;
    }
    val getData()
    {
        return CreateArray(data, 3);
    }
    void setData(float x, float y, float z)
    {
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }
};

int dtMergeCorridorStartMoved(dtPolyRef *path, const int npath, const int maxPath,
                              const dtPolyRef *visited, const size_t nvisited)
{
    int furthestPath = -1;
    int furthestVisited = -1;

    // Find furthest common polygon.
    for (int i = npath - 1; i >= 0; --i)
    {
        bool found = false;
        for (int j = nvisited - 1; j >= 0; --j)
        {
            if (path[i] == visited[j])
            {
                furthestPath = i;
                furthestVisited = j;
                found = true;
            }
        }
        if (found)
            break;
    }

    // If no intersection found just return current path.
    if (furthestPath == -1 || furthestVisited == -1)
        return npath;

    // Concatenate paths.

    // Adjust beginning of the buffer to include the visited.
    const int req = nvisited - furthestVisited;
    const int orig = dtMin(furthestPath + 1, npath);
    int size = dtMax(0, npath - orig);
    if (req + size > maxPath)
        size = maxPath - req;
    if (size)
        memmove(path + req, path + orig, size * sizeof(dtPolyRef));

    // Store visited
    for (int i = 0; i < req; ++i)
        path[i] = visited[(nvisited - 1) - i];

    return req + size;
}

inline val mergeCorridorStartMoved(val path, const int maxPath, val visited)
{
    intptr_t npath;
    dtPolyRef *pathdata = GetArray<dtPolyRef>(path, &npath);
    const std::vector<dtPolyRef> visiteddata = convertJSArrayToNumberVector<dtPolyRef>(visited);
    const size_t nvisited = visiteddata.size();
    int count = dtMergeCorridorStartMoved(pathdata, npath, maxPath, visiteddata.data(), nvisited);
    return CreateArray<dtPolyRef>(pathdata, count);
}
int fixupShortcut(dtPolyRef *path, int npath, dtNavMeshQuery &navQuery)
{
    if (npath < 3)
        return npath;

    // Get connected polygons
    static const int maxNeis = 16;
    dtPolyRef neis[maxNeis];
    int nneis = 0;

    const dtMeshTile *tile = 0;
    const dtPoly *poly = 0;
    if (dtStatusFailed(navQuery.getAttachedNavMesh()->getTileAndPolyByRef(path[0], &tile, &poly)))
        return npath;

    for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
    {
        const dtLink *link = &tile->links[k];
        if (link->ref != 0)
        {
            if (nneis < maxNeis)
                neis[nneis++] = link->ref;
        }
    }

    // If any of the neighbour polygons is within the next few polygons
    // in the path, short cut to that polygon directly.
    static const int maxLookAhead = 6;
    int cut = 0;
    for (int i = dtMin(maxLookAhead, npath) - 1; i > 1 && cut == 0; i--)
    {
        for (int j = 0; j < nneis; j++)
        {
            if (path[i] == neis[j])
            {
                cut = i;
                break;
            }
        }
    }
    if (cut > 1)
    {
        int offset = cut - 1;
        npath -= offset;
        for (int i = 1; i < npath; i++)
            path[i] = path[i + offset];
    }

    return npath;
}

inline val fixupShortcuts(val path, dtNavMeshQuery &navQuery)
{
    intptr_t npath;
    dtPolyRef *data = GetArray<dtPolyRef>(path, &npath);
    int count = fixupShortcut(data, npath, navQuery);
    val outdata = CreateArray<dtPolyRef>(data, count);
    free(data);
    return outdata;
};

struct TitleConfig
{
    dtPolyRef tx;
    dtPolyRef ty;
    float bmin[3];
    float bmax[3];
    int partitionType;
    int triVertexCount;
    int triCount;
    int triFlagCount;
    float agentHeight;
    float agentRadius;
    float agentMaxClimb;
    dtPolyRef triVertexOff;
    dtPolyRef triIndexOff;
    dtPolyRef triFlagOff;
};

enum PartitionType
{
    PARTITION_WATERSHED,
    PARTITION_MONOTONE,
    PARTITION_LAYERS
};

unsigned char *creatNavMeshData(rcConfig m_cfg, TitleConfig titleconfig, int data_ptr, int &dataSize, LayaOffMeshConnection *linkMesh)
{
    double start = emscripten_get_now();
    float *verts = reinterpret_cast<float *>(data_ptr + titleconfig.triVertexOff);
    const int nverts = titleconfig.triVertexCount;
    int *tris = reinterpret_cast<int *>(data_ptr + titleconfig.triIndexOff);
    const int ntris = titleconfig.triCount;
    uint8_t *triFlag = reinterpret_cast<uint8_t *>(data_ptr + titleconfig.triFlagOff);

    // float *offMeshVerts = reinterpret_cast<float *>(data_ptr + titleconfig.offMeshVertsOff);
    // float *offMeshRads = reinterpret_cast<float *>(data_ptr + titleconfig.offMeshRadsOff);
    // unsigned char *offMeshDirs = reinterpret_cast<unsigned char *>(data_ptr + titleconfig.offMeshDirsOff);
    // unsigned char *offMeshAreas = reinterpret_cast<unsigned char *>(data_ptr + titleconfig.offMeshAreasOff);
    // const unsigned short *offMeshFlags = reinterpret_cast<unsigned short *>(data_ptr + titleconfig.offMeshFlagsOff);
    // unsigned int *offMeshIds = reinterpret_cast<unsigned int *>(data_ptr + titleconfig.offMeshIdOff);
    rcContext m_ctx;
    rcVcopy(m_cfg.bmin, titleconfig.bmin);
    rcVcopy(m_cfg.bmax, titleconfig.bmax);
    m_cfg.bmin[0] -= m_cfg.borderSize * m_cfg.cs;
    m_cfg.bmin[2] -= m_cfg.borderSize * m_cfg.cs;
    m_cfg.bmax[0] += m_cfg.borderSize * m_cfg.cs;
    m_cfg.bmax[2] += m_cfg.borderSize * m_cfg.cs;

    rcHeightfield *solid = rcAllocHeightfield();
    if (!solid)
    {
        return 0;
    }
    if (!rcCreateHeightfield(&m_ctx, *solid, m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
    {
        return 0;
    }

    if (!rcRasterizeTriangles(&m_ctx, verts, nverts, tris, triFlag, ntris, *solid, m_cfg.walkableClimb))
    {
        return 0;
    }

    rcFilterLowHangingWalkableObstacles(&m_ctx, m_cfg.walkableClimb, *solid);
    rcFilterLedgeSpans(&m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *solid);
    rcFilterWalkableLowHeightSpans(&m_ctx, m_cfg.walkableHeight, *solid);

    rcCompactHeightfield *m_chf = rcAllocCompactHeightfield();
    if (!m_chf)
    {
        return 0;
    }
    if (!rcBuildCompactHeightfield(&m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *solid, *m_chf))
    {
        return 0;
    }

    rcFreeHeightField(solid);
    solid = 0;

    if (!rcErodeWalkableArea(&m_ctx, m_cfg.walkableRadius, *m_chf))
    {
        return 0;
    }

    if (titleconfig.partitionType == PARTITION_WATERSHED)
    {
        if (!rcBuildDistanceField(&m_ctx, *m_chf))
        {
            return 0;
        }
        if (!rcBuildRegions(&m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
        {
            return 0;
        }
    }
    else if (titleconfig.partitionType == PARTITION_MONOTONE)
    {
        if (!rcBuildRegionsMonotone(&m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
        {
            return 0;
        }
    }
    else // SAMPLE_PARTITION_LAYERS
    {
        // Partition the walkable surface into simple regions without holes.
        if (!rcBuildLayerRegions(&m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea))
        {
            return 0;
        }
    }

    rcContourSet *m_cset = rcAllocContourSet();
    if (!m_cset)
    {
        return 0;
    }
    if (!rcBuildContours(&m_ctx, *m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
    {
        return 0;
    }

    rcPolyMesh *m_pmesh = rcAllocPolyMesh();
    if (!m_pmesh)
    {
        return 0;
    }
    if (!rcBuildPolyMesh(&m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *m_pmesh))
    {
        return 0;
    }

    //
    // Step 7. Create detail mesh which allows to access approximate height on each polygon.
    //

    rcPolyMeshDetail *m_dmesh = rcAllocPolyMeshDetail();
    if (!m_dmesh)
    {
        return 0;
    }

    if (!rcBuildPolyMeshDetail(&m_ctx, *m_pmesh, *m_chf, m_cfg.detailSampleDist, m_cfg.detailSampleMaxError, *m_dmesh))
    {
        return 0;
    }

    rcFreeCompactHeightfield(m_chf);
    m_chf = 0;
    rcFreeContourSet(m_cset);
    m_cset = 0;

    unsigned char *navData = 0;
    int navDataSize = 0;
    if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
    {

        // Update poly flags from areas.
        for (int i = 0; i < m_pmesh->npolys; ++i)
        {
            m_pmesh->flags[i] = m_pmesh->areas[i];
        }

        dtNavMeshCreateParams params;
        memset(&params, 0, sizeof(params));
        params.tileX = titleconfig.tx;
        params.tileY = titleconfig.ty;
        params.verts = m_pmesh->verts;
        params.vertCount = m_pmesh->nverts;
        params.polys = m_pmesh->polys;
        params.polyAreas = m_pmesh->areas;
        params.polyFlags = m_pmesh->flags;
        params.polyCount = m_pmesh->npolys;
        params.nvp = m_pmesh->nvp;
        params.detailMeshes = m_dmesh->meshes;
        params.detailVerts = m_dmesh->verts;
        params.detailVertsCount = m_dmesh->nverts;
        params.detailTris = m_dmesh->tris;
        params.detailTriCount = m_dmesh->ntris;
        params.offMeshConVerts = linkMesh->m_offMeshConVerts;
        params.offMeshConRad = linkMesh->m_offMeshConRads;
        params.offMeshConDir = linkMesh->m_offMeshConDirs;
        params.offMeshConAreas = linkMesh->m_offMeshConAreas;
        params.offMeshConFlags = linkMesh->m_offMeshConFlags;
        params.offMeshConUserID = linkMesh->m_offMeshConId;
        params.offMeshConCount = linkMesh->m_offMeshConCount;
        params.walkableHeight = titleconfig.agentHeight;
        params.walkableRadius = titleconfig.agentRadius;
        params.walkableClimb = titleconfig.agentMaxClimb;
        rcVcopy(params.bmin, m_pmesh->bmin);
        rcVcopy(params.bmax, m_pmesh->bmax);
        params.cs = m_cfg.cs;
        params.ch = m_cfg.ch;
        params.buildBvTree = true;

        if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
        {
            return 0;
        }
    }

    dataSize = navDataSize;
    // printf("We waited long enough, %f ms\n", emscripten_get_now() - start);
    return navData;
}

EMSCRIPTEN_BINDINGS(recastnavigation)
{
    function("dtStatusSucceed", dtStatusSucceed);
    function("dtStatusFailed", dtStatusFailed);
    function("fixupShortcuts", fixupShortcuts);
    function("mergeCorridorStartMoved", mergeCorridorStartMoved);
    function("dtFreeNavMeshQuery", dtFreeNavMeshQuery, allow_raw_pointers());
    function("dtFreeNavMesh", dtFreeNavMesh, allow_raw_pointers());
    function("dtFreeCrowd", dtFreeCrowd, allow_raw_pointers());

    value_array<std::array<float, 3>>("array_float_3")
        .element(emscripten::index<0>())
        .element(emscripten::index<1>())
        .element(emscripten::index<2>());
    class_<LayaOffMeshConnection>("dtOffMeshConnection")
        .constructor<>()
        .function("addOffMeshConnection", &LayaOffMeshConnection::addOffMeshConnection, allow_raw_pointers())
        .function("deleteOffMeshConnection", &LayaOffMeshConnection::deleteOffMeshConnection);

    value_object<rcConfig>("rcConfig")
        .field("width", &rcConfig::width)
        .field("height", &rcConfig::height)
        .field("tileSize", &rcConfig::tileSize)
        .field("borderSize", &rcConfig::borderSize)
        .field("cs", &rcConfig::cs)
        .field("ch", &rcConfig::ch)
        .field("walkableSlopeAngle", &rcConfig::walkableSlopeAngle)
        .field("walkableHeight", &rcConfig::walkableHeight)
        .field("walkableClimb", &rcConfig::walkableClimb)
        .field("walkableRadius", &rcConfig::walkableRadius)
        .field("maxEdgeLen", &rcConfig::maxEdgeLen)
        .field("maxSimplificationError", &rcConfig::maxSimplificationError)
        .field("minRegionArea", &rcConfig::minRegionArea)
        .field("mergeRegionArea", &rcConfig::mergeRegionArea)
        .field("maxVertsPerPoly", &rcConfig::maxVertsPerPoly)
        .field("detailSampleDist", &rcConfig::detailSampleDist)
        .field("detailSampleMaxError", &rcConfig::detailSampleMaxError);

    value_object<TitleConfig>("TitleConfig")
        .field("tx", &TitleConfig::tx)
        .field("ty", &TitleConfig::ty)
        .field("bmin", &TitleConfig::bmin)
        .field("bmax", &TitleConfig::bmax)
        .field("partitionType", &TitleConfig::partitionType)
        .field("triCount", &TitleConfig::triCount)
        .field("triVertexCount", &TitleConfig::triVertexCount)
        .field("triFlagCount", &TitleConfig::triFlagCount)
        .field("agentHeight", &TitleConfig::agentHeight)
        .field("agentRadius", &TitleConfig::agentRadius)
        .field("agentMaxClimb", &TitleConfig::agentMaxClimb)
        .field("triVertexOff", &TitleConfig::triVertexOff)
        .field("triIndexOff", &TitleConfig::triIndexOff)
        .field("triFlagOff", &TitleConfig::triFlagOff);

    enum_<dtStraightPathFlags>("dtStraightPathFlags")
        .value("DT_STRAIGHTPATH_START", DT_STRAIGHTPATH_START)
        .value("DT_STRAIGHTPATH_END", DT_STRAIGHTPATH_END)
        .value("DT_STRAIGHTPATH_OFFMESH_CONNECTION", DT_STRAIGHTPATH_OFFMESH_CONNECTION);

    class_<dtMeshHeader>("dtMeshHeader")
        .constructor<>()
        .property("magic", &dtMeshHeader::magic)
        .property("x", &dtMeshHeader::x)
        .property("y", &dtMeshHeader::y)
        .property("layer", &dtMeshHeader::layer)
        .property("userId", &dtMeshHeader::userId)
        .property("polyCount", &dtMeshHeader::polyCount)
        .property("vertCount", &dtMeshHeader::vertCount)
        .property("maxLinkCount", &dtMeshHeader::maxLinkCount)
        .property("detailMeshCount", &dtMeshHeader::detailMeshCount)
        .property("detailVertCount", &dtMeshHeader::detailVertCount)

        .property("detailTriCount", &dtMeshHeader::detailTriCount)
        .property("walkableHeight", &dtMeshHeader::walkableHeight)
        .property("walkableRadius", &dtMeshHeader::walkableRadius)
        .property("walkableClimb", &dtMeshHeader::vertCount)
        .function("boundMax", optional_override([](const dtMeshHeader *header)
                                                { return CreateArray<float>(header->bmax, 3); }),
                  allow_raw_pointers())
        .function("boundMin", optional_override([](const dtMeshHeader *header)
                                                { return CreateArray<float>(header->bmin, 3); }),
                  allow_raw_pointers());

    class_<dtPoly>("dtPoly")
        .property("firstLink", &dtPoly::firstLink)
        .property("flags", &dtPoly::flags)
        .property("vertCount", &dtPoly::vertCount)
        .property("areaAndtype", &dtPoly::areaAndtype)
        .function("getVerts", optional_override([](dtPoly &poly)
                                                { return CreateArrayView<unsigned short>(poly.verts, poly.vertCount); }))
        .function("getType", &dtPoly::getType)
        .function("getArea", &dtPoly::getArea)
        .function("getNeis", optional_override([](dtPoly &poly, int i)
                                               { return poly.neis[i]; }));
    class_<dtPolyDetail>("dtPolyDetail")
        .property("vertBase", &dtPolyDetail::vertBase)
        .property("triBase", &dtPolyDetail::triBase)
        .property("vertCount", &dtPolyDetail::vertCount)
        .property("triCount", &dtPolyDetail::triCount);

    class_<dtMeshTile>("dtMeshTile")
        .function("getheader", optional_override([](const dtMeshTile *title)
                                                 { return title->header; }),
                  allow_raw_pointers())
        .function("getVerts", optional_override([](const dtMeshTile *title)
                                                { return CreateArrayView<float>(title->verts, 3 * title->header->vertCount); }),
                  allow_raw_pointers())
        .function("getPolys", optional_override([](const dtMeshTile *title, int i)
                                                { return &title->polys[i]; }),
                  allow_raw_pointers())
        .function("getPolyDetail", optional_override([](const dtMeshTile *title, int i)
                                                     { return &title->detailMeshes[i]; }),
                  allow_raw_pointers())
        .function("getdetailTris", optional_override([](const dtMeshTile *title)
                                                     { return CreateArrayView<unsigned char>(title->detailTris, 4 * title->header->detailTriCount); }),
                  allow_raw_pointers())
        .function("getdetailVerts", optional_override([](const dtMeshTile *title)
                                                      { return CreateArrayView<float>(title->detailVerts, 3 * title->header->detailVertCount); }),
                  allow_raw_pointers());

    class_<dtCrowdAgentParams>("dtCrowdAgentParams")
        .constructor<>()
        .property("radius", &dtCrowdAgentParams::radius)
        .property("height", &dtCrowdAgentParams::height)
        .property("maxAcceleration", &dtCrowdAgentParams::maxAcceleration)
        .property("maxSpeed", &dtCrowdAgentParams::maxSpeed)
        .property("collisionQueryRange", &dtCrowdAgentParams::collisionQueryRange)
        .property("pathOptimizationRange", &dtCrowdAgentParams::pathOptimizationRange)
        .property("separationWeight", &dtCrowdAgentParams::separationWeight)
        .property("updateFlags", &dtCrowdAgentParams::updateFlags)
        .property("obstacleAvoidanceType", &dtCrowdAgentParams::obstacleAvoidanceType)
        .property("queryFilterType", &dtCrowdAgentParams::queryFilterType);

    class_<dtCrowdAgent>("dtCrowdAgent")
        .constructor<>()
        .property("active", &dtCrowdAgent::active)
        .property("state", &dtCrowdAgent::state)
        .function("getparams", optional_override([](const dtCrowdAgent *agent)
                                                 { return &agent->params; }),
                  allow_raw_pointers())
        .property("targetState", &dtCrowdAgent::targetState)
        .property("targetRef", &dtCrowdAgent::targetRef)
        .function("getCurPos", optional_override([](const dtCrowdAgent *agent, val out)
                                                 { writeToJsObject(out, &agent->npos[0]); }),
                  allow_raw_pointers())
        .function("getCurDir", optional_override([](const dtCrowdAgent *agent, val out)
                                                 { writeToJsObject(out, &agent->dvel[0]); }),
                  allow_raw_pointers());

    class_<dtCrowd>("dtCrowd")
        .constructor<>()
        .function("init", &dtCrowd::init, allow_raw_pointers())
        .function("addAgent", optional_override([](dtCrowd &btcrowd, std::array<float, 3> pos, dtCrowdAgentParams &params)
                                                { return btcrowd.addAgent(pos.data(), &params); }))
        .function("removeAgent", optional_override([](dtCrowd &btcrowd, const int idx)
                                                   { return btcrowd.removeAgent(idx); }))
        .function("getAgent", &dtCrowd::getAgent, allow_raw_pointers())
        .function("getAgentCount", &dtCrowd::getAgentCount)
        .function("requestMoveTarget", optional_override([](dtCrowd &btcrowd, const int idx, dtPolyRef ref, std::array<float, 3> pos)
                                                         { return btcrowd.requestMoveTarget(idx, ref, pos.data()); }))
        .function("requestMoveVelocity", optional_override([](dtCrowd &btcrowd, const int idx, std::array<float, 3> veldata)
                                                           { return btcrowd.requestMoveVelocity(idx, veldata.data()); }))
        .function("getFilter", optional_override([](dtCrowd &btcrowd, const int i)
                                                 { return btcrowd.getEditableFilter(i); }),
                  allow_raw_pointers())
        .function("update", optional_override([](dtCrowd &btcrowd, const float dt)
                                              { btcrowd.update(dt, nullptr); }));

    class_<dtRefPointData>("dtRefPointData")
        .constructor<>()
        .function("getployRef", &dtRefPointData::getployRef)
        .function("getx", &dtRefPointData::getx)
        .function("gety", &dtRefPointData::gety)
        .function("getz", &dtRefPointData::getz)
        .function("setx", &dtRefPointData::setx)
        .function("sety", &dtRefPointData::sety)
        .function("setz", &dtRefPointData::setz)
        .function("getData", &dtRefPointData::getData)
        .function("setData", &dtRefPointData::setData);

    register_vector<float>("dtFloatVector");

    register_vector<dtPolyRef>("dtPolyRefVector");

    value_object<dtNavMeshParams>("dtNavMeshParams")
        .field("orig", &dtNavMeshParams::orig)
        .field("tileWidth", &dtNavMeshParams::tileWidth)
        .field("tileHeight", &dtNavMeshParams::tileHeight)
        .field("maxTiles", &dtNavMeshParams::maxTiles)
        .field("maxPolys", &dtNavMeshParams::maxPolys);

    class_<dtNavMesh>("dtNavMesh")
        .constructor<>()
        .function("init", optional_override([](dtNavMesh &dtnaveMesh, dtNavMeshParams data)
                                            { dtnaveMesh.init(&data); }))
        .function("init", optional_override([](dtNavMesh &dtnaveMesh, int data_ptr, int size)
                                            { 
                                                unsigned char *data = reinterpret_cast<unsigned char *>(data_ptr);
                                                dtnaveMesh.init(data, size, DT_TILE_FREE_DATA); }))
        .function("getMaxTiles", &dtNavMesh::getMaxTiles)
        .function("addTile", optional_override([](dtNavMesh &dtnaveMesh, rcConfig m_cfg, TitleConfig titleconfig, int data_ptr, LayaOffMeshConnection *linkMesh)
                                               { 
                                                    int navDataSize = 0;
                                                    unsigned char *navData = creatNavMeshData(m_cfg, titleconfig, data_ptr, navDataSize, linkMesh);
                                                    if (navData){
                                                        dtnaveMesh.addTile(navData, navDataSize, DT_TILE_FREE_DATA, 0, 0);
                                                    }else
                                                    {
                                                        printf("fail tile\n");
                                                    } }),
                  allow_raw_pointers())
        .function("getTile", select_overload<const dtMeshTile *(int) const>(&dtNavMesh::getTile), allow_raw_pointers())
        .function("getTileRefAt", &dtNavMesh::getTileRefAt)
        .function("removeTile", optional_override([](dtNavMesh &dtnaveMesh, const int tx, const int ty)
                                                  { dtnaveMesh.removeTile(dtnaveMesh.getTileRefAt(tx, ty, 0), 0, 0); }))
        .function("getOffMeshConnectionPolyEndPoints", optional_override([](dtNavMesh &dtnaveMesh, dtPolyRef prevRef, dtPolyRef polyRef,
                                                                            val startPos, val endPos)
                                                                         {
                                                                            intptr_t startSize = 0;
                                                                            float *startPosData = GetArray<float>(startPos, &startSize);
                                                                            intptr_t endSize = 0;
                                                                            float *endPosData = GetArray<float>(endPos, &endSize);
                                                                            dtStatus status = dtnaveMesh.getOffMeshConnectionPolyEndPoints(prevRef, polyRef,startPosData, endPosData);
                                                                            writeToArray(startPos, startPosData, startSize);
                                                                            free(startPosData);
                                                                            writeToArray(endPos, endPosData, endSize);
                                                                            free(endPosData);
                                                                            return status; }))
        .function("getMaxTiles", &dtNavMesh::getMaxTiles);

    class_<dtQueryFilter>("dtQueryFilter")
        .constructor<>()
        .function("getAreaCost", &dtQueryFilter::getAreaCost)
        .function("setAreaCost", &dtQueryFilter::setAreaCost)
        .function("getIncludeFlags", &dtQueryFilter::getIncludeFlags)
        .function("setIncludeFlags", &dtQueryFilter::setIncludeFlags)
        .function("getExcludeFlags", &dtQueryFilter::getExcludeFlags)
        .function("setExcludeFlags", &dtQueryFilter::setExcludeFlags);

    class_<dtNavMeshQuery>("dtNavMeshQuery")
        .constructor<>()
        .function("init", &dtNavMeshQuery::init, allow_raw_pointers())
        .function("getPolyHeight", optional_override([](dtNavMeshQuery &dtnavMeshQuery, dtPolyRef ref, std::array<float, 3> vecData)
                                                     {
                                                        float h = 0;
                                                        dtStatus status = dtnavMeshQuery.getPolyHeight(ref, vecData.data(), &h);
                                                        auto out = val::object();
                                                        out.set("height", val(h));
                                                        out.set("Status", val(status));
                                                        return out ; }))
        .function("findPath", optional_override([](dtNavMeshQuery &dtnavMeshQuery,
                                                   dtRefPointData &startdata, dtRefPointData &enddata, dtQueryFilter &filter_ptr, int MAX_POLYS)
                                                {
                                                        const dtQueryFilter* filter = (const dtQueryFilter*)&filter_ptr;
                                                        dtPolyRef m_polys[MAX_POLYS];
                                                        int pathCount = 0;
                                                        dtStatus status = dtnavMeshQuery.findPath(startdata.dtpolyRef, enddata.dtpolyRef, startdata.data, enddata.data, filter, m_polys, &pathCount, MAX_POLYS);
                                                        auto out = val::object();
                                                        out.set("Status", val(status));
                                                        out.set("polys", CreateArray<dtPolyRef>(&m_polys[0], pathCount));
                                                        return out ; }))
        .function("closestPointOnPoly", optional_override([](dtNavMeshQuery &dtnavMeshQuery, dtPolyRef ref, val data)
                                                          {
                                                        const std::vector<float>& vecData = convertJSArrayToNumberVector<float>(data);
                                                        float iterPos[3];
                                                        dtnavMeshQuery.closestPointOnPoly(ref, vecData.data(), iterPos, 0);
                                                        return CreateArray<float>(&iterPos[0], 3); }))
        .function("closestPointOnPolyByRefPointData", optional_override([](dtNavMeshQuery &dtnavMeshQuery, dtRefPointData *refdata)
                                                                        {
                                                        float iterPos[3];
                                                        dtnavMeshQuery.closestPointOnPoly(refdata->dtpolyRef, refdata->data, iterPos, 0);
                                                        return CreateArray<float>(&iterPos[0], 3); }),
                  allow_raw_pointers())
        .function("findStraightPath", optional_override([](dtNavMeshQuery &dtnavMeshQuery,
                                                           std::array<float, 3> startPos, std::array<float, 3> endPos,
                                                           val path, int pathSize, const int maxStraightPath)
                                                        {
                                                            float steerPath[maxStraightPath*3];
                                                            unsigned char steerPathFlags[maxStraightPath];
                                                            dtPolyRef steerPathPolys[maxStraightPath];
                                                            int nsteerPath = 0;
                                                            const std::vector<dtPolyRef> &pathData = convertJSArrayToNumberVector<dtPolyRef>(path);
                                                            dtnavMeshQuery.findStraightPath(startPos.data(), endPos.data(), pathData.data(), pathSize,
                                                                                            steerPath, steerPathFlags, steerPathPolys, &nsteerPath, maxStraightPath);
                                                            auto out = val::object();
                                                            out.set("steerPath", CreateArray<float>(&steerPath[0], maxStraightPath * 3));
                                                            out.set("steerPathPolys", CreateArray<dtPolyRef>(&steerPathPolys[0], maxStraightPath));
                                                            out.set("steerPathFlags", CreateArray<unsigned char>(&steerPathFlags[0], maxStraightPath));
                                                            out.set("nsteerPath", val(nsteerPath));
                                                            return out; }))
        .function("moveAlongSurface", optional_override([](dtNavMeshQuery &dtnavMeshQuery, dtPolyRef ref,
                                                           std::array<float, 3> startPos, std::array<float, 3> endPos,
                                                           dtQueryFilter &filter_ptr, const int maxVisitedSize)
                                                        {
                                                            const dtQueryFilter* filter = (const dtQueryFilter*)&filter_ptr;
                                                            float result[3];
                                                            dtPolyRef visited[maxVisitedSize];
                                                            int nvisited = 0;
                                                            dtStatus status = dtnavMeshQuery.moveAlongSurface(ref, startPos.data(), endPos.data(), filter, result, visited, &nvisited, maxVisitedSize);
                                                            auto out = val::object();
                                                            out.set("resultPos", CreateArray<float>(&result[0], 3));
                                                            out.set("visited", CreateArrayView<dtPolyRef>(&visited[0], nvisited));
                                                            out.set("status", val(status));
                                                            return out; }))
        .function("findNearestPoly", optional_override([](dtNavMeshQuery &dtnavMeshQuery,
                                                          dtRefPointData &pointdata, std::array<float, 3> halfExtents, dtQueryFilter &filter_ptr)
                                                       {
                                                        const dtQueryFilter* filter = (const dtQueryFilter*)&filter_ptr;
                                                        return dtnavMeshQuery.findNearestPoly(pointdata.data, halfExtents.data(), filter, &pointdata.dtpolyRef, 0); }))
        .function("findRandomPoint", optional_override([](dtNavMeshQuery &dtnavMeshQuery, dtQueryFilter &filter_ptr, dtRefPointData &randomdata)
                                                       {
                                                        const dtQueryFilter* filter = (const dtQueryFilter*)&filter_ptr;
                                                        return dtnavMeshQuery.findRandomPoint(filter, frand, &randomdata.dtpolyRef, randomdata.data); }))
        .function("findRandomPointAroundCircle", optional_override([](dtNavMeshQuery &dtnavMeshQuery,
                                                                      dtRefPointData &startPoint, float maxRadius,
                                                                      dtQueryFilter &filter_ptr, dtRefPointData &randomdata)
                                                                   {
                                                                    const dtQueryFilter* filter = (const dtQueryFilter*)&filter_ptr;
                                                                    return dtnavMeshQuery.findRandomPointAroundCircle(startPoint.dtpolyRef, startPoint.data, maxRadius, filter, frand, &randomdata.dtpolyRef, randomdata.data); }));
}
