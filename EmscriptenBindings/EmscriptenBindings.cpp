
#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <emscripten/html5.h>

#include <stdlib.h>
#include <stdio.h>
#include <cstdint>
#include <cstring>
#include <math.h>

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "DetourCrowd.h"
#include "DetourNavMeshBuilder.h"
#include "Recast.h"
#include "LayaOffMeshConnection.h"
#include "NavTileCache.h"
#include "LayaConvexVolume.h"

using namespace emscripten;

typedef std::array<float, 3> Vec3;

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

    float agentHeight;
    float agentRadius;
    float agentMaxClimb;
    float maxSimplificationError;
    int maxEdgeLen;
};

inline unsigned int nextPow2(unsigned int v)
{
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;
    return v;
}

inline unsigned int ilog2(unsigned int v)
{
    unsigned int r;
    unsigned int shift;
    r = (v > 0xffff) << 4;
    v >>= r;
    shift = (v > 0xff) << 3;
    v >>= shift;
    r |= shift;
    shift = (v > 0xf) << 2;
    v >>= shift;
    r |= shift;
    shift = (v > 0x3) << 1;
    v >>= shift;
    r |= shift;
    r |= (v >> 1);
    return r;
}

enum PartitionType
{
    PARTITION_WATERSHED,
    PARTITION_MONOTONE,
    PARTITION_LAYERS
};

inline void printMsg(char const *msg, float *time){
    float now = emscripten_get_now();
    printf(" __ %s: %f \n",msg,now - *time);
    *time = now;
}

unsigned char *creatNavMeshData(rcConfig m_cfg, TitleConfig titleconfig, val datas, int &dataSize, LayaOffMeshConnection *linkMesh, LayaConvexVolume *volumes)
{
    float now = emscripten_get_now();
    m_cfg.walkableClimb = dtMax((int)ceilf(titleconfig.agentMaxClimb / m_cfg.ch), 1);
    m_cfg.walkableHeight = (int)ceilf(titleconfig.agentHeight / m_cfg.ch);
    m_cfg.walkableRadius = (int)ceilf(titleconfig.agentRadius / m_cfg.cs);
    m_cfg.maxEdgeLen = titleconfig.maxEdgeLen;
    m_cfg.minRegionArea = 64;
    m_cfg.mergeRegionArea = 40000;
    m_cfg.borderSize = m_cfg.walkableRadius + 3;
    m_cfg.width = m_cfg.tileSize + m_cfg.borderSize * 2;
    m_cfg.height = m_cfg.tileSize + m_cfg.borderSize * 2;
    m_cfg.maxSimplificationError = titleconfig.maxSimplificationError;
    m_cfg.detailSampleDist = m_cfg.cs * 6.0f;
    m_cfg.detailSampleMaxError = m_cfg.ch;
    m_cfg.maxVertsPerPoly = 6.0f;

    rcContext m_ctx;
    rcVcopy(m_cfg.bmin, titleconfig.bmin);
    rcVcopy(m_cfg.bmax, titleconfig.bmax);
    const float border = m_cfg.borderSize * m_cfg.cs;
    m_cfg.bmin[0] -= border;
    m_cfg.bmin[2] -= border;
    m_cfg.bmax[0] += border;
    m_cfg.bmax[2] += border;

    const std::vector<int> vids = vecFromJSArray<int>(datas);
    std::vector<NavTileCache *> dataVec;
    int nverts = 0;
    int ntris = 0;
    int triFlagCount = 0;
    for (int i = 0; i < vids.size(); i++)
    {
        NavTileCache *iter = (NavTileCache *)vids[i];
        dataVec.push_back(iter);
        nverts += iter->getTriVertexCount();
        ntris += iter->getTriIndexCount();
        triFlagCount += iter->getTriFlagCount();
    }

    float *verts = new float[nverts];
    int *tris = new int[ntris];
    uint8_t *triFlag = new uint8_t[triFlagCount];
    nverts = 0;
    ntris = 0;
    triFlagCount = 0;
    for (auto iter : dataVec)
    {
        memcpy(&verts[nverts], iter->getVerts(), sizeof(float) * iter->getTriVertexCount());
        memcpy(&triFlag[triFlagCount], iter->getFlags(), sizeof(uint8_t) * iter->getTriFlagCount());
        memcpy(&tris[ntris], iter->getTris(), sizeof(int) * iter->getTriIndexCount());
        int vertsCount = nverts / 3;
        if (vertsCount > 0)
        {
            int triCount = iter->getTriIndexCount();
            for (int i = 0; i < triCount; i++)
            {
                tris[ntris + i] += vertsCount;
            }
        }
        nverts += iter->getTriVertexCount();
        triFlagCount += iter->getTriFlagCount();
        ntris += iter->getTriIndexCount();
    }
    ntris /= 3;
    printMsg("creatNavMeshData", &now);
    
    rcHeightfield *solid = rcAllocHeightfield();
    if (!solid)
    {
        return 0;
    }
    if (!rcCreateHeightfield(&m_ctx, *solid, m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
    {
        return 0;
    }

    printMsg("rcCreateHeightfield", &now);
    
    if (!rcRasterizeTriangles(&m_ctx, verts, nverts, tris, triFlag, ntris, *solid, m_cfg.walkableClimb))
    {
        return 0;
    }

    printMsg("rcRasterizeTriangles", &now);

    delete[] verts;
    delete[] tris;
    delete[] triFlag;
    verts = 0;
    tris = 0;
    triFlag = 0;

    rcFilterLowHangingWalkableObstacles(&m_ctx, m_cfg.walkableClimb, *solid);
    rcFilterLedgeSpans(&m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *solid);
    rcFilterWalkableLowHeightSpans(&m_ctx, m_cfg.walkableHeight, *solid);

    printMsg("rcFilterLowHangingWalkableObstacles", &now);  
 
    rcCompactHeightfield *m_chf = rcAllocCompactHeightfield();
    if (!m_chf)
    {
        return 0;
    }
    if (!rcBuildCompactHeightfield(&m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *solid, *m_chf))
    {
        return 0;
    }

    printMsg("rcBuildCompactHeightfield", &now);

    rcFreeHeightField(solid);
    solid = 0;

    if (!rcErodeWalkableArea(&m_ctx, m_cfg.walkableRadius, *m_chf))
    {
        return 0;
    }

    printMsg("rcErodeWalkableArea", &now);

    if (volumes->is3D)
    {
        for (int i = 0; i < volumes->m_ConvexCount; ++i)
        {
            rcMarkConvexVolume(&m_ctx, volumes, i, *m_chf);
        }
    }
    else
    {
        const ConvexVolume *vols = volumes->m_volumes;
        for (int i = 0; i < volumes->m_ConvexCount; ++i)
        {
            rcMarkConvexPolyArea(&m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_chf);
        }
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

    printMsg("rcBuildRegions", &now);

    rcContourSet *m_cset = rcAllocContourSet();
    if (!m_cset)
    {
        return 0;
    }
    if (!rcBuildContours(&m_ctx, *m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
    {
        return 0;
    }

    printMsg("rcBuildContours", &now);
   

    rcPolyMesh *m_pmesh = rcAllocPolyMesh();
    if (!m_pmesh)
    {
        return 0;
    }
    if (!rcBuildPolyMesh(&m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *m_pmesh))
    {
        return 0;
    }

    printMsg("rcBuildPolyMesh", &now);
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

    printMsg("rcBuildPolyMeshDetail", &now);

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
            m_pmesh->flags[i] = 1 << int(m_pmesh->areas[i] - 1);
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
        params.walkableClimb = m_cfg.walkableClimb * m_cfg.ch;
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

    printMsg("dtCreateNavMeshData", &now);

    dataSize = navDataSize;
    return navData;
}

inline val updateCrowd(dtCrowd *crowd, const float dt)
{
    crowd->update(dt, 0);
    int agentCount = crowd->getAgentCount();
    float buffer[agentCount * 6];
    for (int i = 0; i < agentCount; i++)
    {
        const dtCrowdAgent *agent = crowd->getAgent(i);
        const float *pos = agent->npos;
        const float *dvel = agent->dvel;
        buffer[i * 6 + 0] = pos[0];
        buffer[i * 6 + 1] = pos[1];
        buffer[i * 6 + 2] = pos[2];
        buffer[i * 6 + 3] = dvel[0];
        buffer[i * 6 + 4] = dvel[1];
        buffer[i * 6 + 5] = dvel[2];
    }
    return CreateArrayView<float>(buffer, agentCount * 6);
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
    function("dtFree", dtFree, allow_raw_pointers());
    function("updateCrowd", updateCrowd, allow_raw_pointers());

    value_array<Vec3>("Vec3")
        .element(emscripten::index<0>())
        .element(emscripten::index<1>())
        .element(emscripten::index<2>());

    value_object<dtRefPointData>("dtRefPointData")
        .field("polyRef", &dtRefPointData::dtpolyRef)
        .field("data", &dtRefPointData::data);

    class_<LayaOffMeshConnection>("dtOffMeshConnection")
        .constructor<>()
        .function("addOffMeshConnection", &LayaOffMeshConnection::addOffMeshConnection, allow_raw_pointers())
        .function("deleteOffMeshConnection", &LayaOffMeshConnection::deleteOffMeshConnection);

    class_<LayaConvexVolume>("dtConvexVolume")
        .constructor<>()
        .function("setIs3D", &LayaConvexVolume::setIs3D)
        .function("addCovexVoume", optional_override([](LayaConvexVolume *volume, int id, val data, const float minh, const float maxh, unsigned char area)
                                                     {
                                                            const std::vector<float> dataVec = convertJSArrayToNumberVector<float>(data);
                                                            return volume->addCovexVoume(id,dataVec.data(),dataVec.size(),minh,maxh,area); }),
                  allow_raw_pointers())
        // .function("updateCovexVoume", optional_override([](LayaConvexVolume *volume, int id, val data, const float minh, const float maxh,  unsigned char area)
        //                                                 {
        //                                                     const std::vector<float> dataVec = convertJSArrayToNumberVector<float>(data);
        //                                                      int nverts = (int)(dataVec.size()/3);
        //                                                     volume->updateCovexVoume(id, dataVec.data(), nverts,minh,maxh, area); }),
        //           allow_raw_pointers())
        .function("deleteCovexVoume", &LayaConvexVolume::deleteCovexVoume);

    class_<NavTileData>("dtNavTileData")
        .constructor<>()
        .function("setTriVertex", optional_override([](NavTileData *titleData, val data)
                                                    { const std::vector<float> dataVec = convertJSArrayToNumberVector<float>(data);
                                                        titleData->setTriVertex(dataVec.data(),dataVec.size()); }),
                  allow_raw_pointers())
        .function("setTriIndex", optional_override([](NavTileData *titleData, val data)
                                                   { const std::vector<int> dataVec = convertJSArrayToNumberVector<int>(data);
                                                    titleData->setTriIndex(dataVec.data(), dataVec.size()); }),
                  allow_raw_pointers())
        .function("setTriFlag", optional_override([](NavTileData *titleData, val data)
                                                  { const std::vector<uint8_t> dataVec = convertJSArrayToNumberVector<uint8_t>(data);
                                                    titleData->setTriFlag(dataVec.data(), dataVec.size()); }),
                  allow_raw_pointers())
        .function("destroy", optional_override([](NavTileData *titleData)
                                               { titleData->~NavTileData(); }),
                  allow_raw_pointers());

    class_<NavTileCache>("dtNavTileCache")
        .constructor<>()
        .function("init", &NavTileCache::init, allow_raw_pointers())
        .function("setFlag", &NavTileCache::setFlag)
        .function("getTriIndexCount", &NavTileCache::getTriIndexCount)
        .function("getTriVertexCount", &NavTileCache::getTriVertexCount)
        .function("getVerts", optional_override([](NavTileCache *cache)
                                                { return CreateArrayView<float>(cache->getVerts(), cache->getTriVertexCount()); }),
                  allow_raw_pointers())
        .function("getTris", optional_override([](NavTileCache *cache)
                                               { return CreateArrayView<int>(cache->getTris(), cache->getTriIndexCount()); }),
                  allow_raw_pointers())
        .function("transfromData", optional_override([](NavTileCache *cache, val data)
                                                     { const std::vector<float> dataVec = convertJSArrayToNumberVector<float>(data);
                                                        cache->transfromData(dataVec.data()); }),
                  allow_raw_pointers())
        .function("destroy", optional_override([](NavTileCache *cache)
                                               { cache->~NavTileCache(); }),
                  allow_raw_pointers());

    value_object<rcConfig>("rcConfig")
        .field("tileSize", &rcConfig::tileSize)
        .field("cellSize", &rcConfig::cs)
        .field("cellHeight", &rcConfig::ch)
        .field("agentMaxSlope", &rcConfig::walkableSlopeAngle);

    value_object<TitleConfig>("TitleConfig")
        .field("tx", &TitleConfig::tx)
        .field("ty", &TitleConfig::ty)
        .field("bmin", &TitleConfig::bmin)
        .field("bmax", &TitleConfig::bmax)
        .field("partitionType", &TitleConfig::partitionType)
        .field("agentHeight", &TitleConfig::agentHeight)
        .field("agentRadius", &TitleConfig::agentRadius)
        .field("agentMaxClimb", &TitleConfig::agentMaxClimb)
        .field("maxEdgeLen", &TitleConfig::maxEdgeLen)
        .field("maxSimplificationError", &TitleConfig::maxSimplificationError);

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
        // .function("getNeis", optional_override([](dtPoly &poly)
        //                                        { return CreateArrayView<unsigned short>(poly.neis, poly.vertCount); }));
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
        .function("init", optional_override([](dtCrowd &crowd, const int maxAgents, const float maxAgentRadius, dtNavMesh *nav)
                                            { bool res = crowd.init(maxAgents, maxAgentRadius, nav);
                                                // Setup local avoidance params to different qualities.
                                                dtObstacleAvoidanceParams params;
                                                // Use mostly default settings, copy from dtCrowd.
                                                memcpy(&params, crowd.getObstacleAvoidanceParams(0), sizeof(dtObstacleAvoidanceParams));

                                                // Low (11)
                                                params.velBias = 0.5f;
                                                params.adaptiveDivs = 5;
                                                params.adaptiveRings = 2;
                                                params.adaptiveDepth = 1;
                                                crowd.setObstacleAvoidanceParams(0, &params);

                                                // Medium (22)
                                                params.velBias = 0.5f;
                                                params.adaptiveDivs = 5;
                                                params.adaptiveRings = 2;
                                                params.adaptiveDepth = 2;
                                                crowd.setObstacleAvoidanceParams(1, &params);

                                                // Good (45)
                                                params.velBias = 0.5f;
                                                params.adaptiveDivs = 7;
                                                params.adaptiveRings = 2;
                                                params.adaptiveDepth = 3;
                                                crowd.setObstacleAvoidanceParams(2, &params);

                                                // High (66)
                                                params.velBias = 0.5f;
                                                params.adaptiveDivs = 7;
                                                params.adaptiveRings = 3;
                                                params.adaptiveDepth = 3;

                                                crowd.setObstacleAvoidanceParams(3, &params);
                                                return res; }),
                  allow_raw_pointers())
        .function("addAgent", optional_override([](dtCrowd &btcrowd, Vec3 pos, dtCrowdAgentParams &params)
                                                { return btcrowd.addAgent(pos.data(), &params); }))
        .function("removeAgent", optional_override([](dtCrowd &btcrowd, const int idx)
                                                   { return btcrowd.removeAgent(idx); }))
        .function("getAgent", &dtCrowd::getAgent, allow_raw_pointers())
        .function("getAgentCount", &dtCrowd::getAgentCount)
        .function("requestMoveTarget", optional_override([](dtCrowd &btcrowd, const int idx, dtRefPointData point)
                                                         { return btcrowd.requestMoveTarget(idx, point.dtpolyRef, point.data); }))
        .function("requestMoveVelocity", optional_override([](dtCrowd &btcrowd, const int idx, Vec3 veldata)
                                                           { return btcrowd.requestMoveVelocity(idx, veldata.data()); }))
        .function("getFilter", optional_override([](dtCrowd &btcrowd, const int i)
                                                 { return btcrowd.getEditableFilter(i); }),
                  allow_raw_pointers())
        .function("update", optional_override([](dtCrowd &btcrowd, const float dt)
                                              { btcrowd.update(dt, nullptr); }));

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
        .function("init", optional_override([](dtNavMesh &dtnaveMesh, Vec3 min, Vec3 max, float cellsize, int tileSize)
                                            {
                                                int gw = 0, gh = 0;
                                                const float *bmin = min.data();
                                                const float *bmax = max.data();
                                                rcCalcGridSize(bmin, bmax, cellsize, &gw, &gh);
                                                const int tw = (gw + tileSize - 1) / tileSize;
                                                const int th = (gh + tileSize - 1) / tileSize;
                                                int tileBits = rcMin((int)ilog2(nextPow2(tw * th)), 14);
                                                if (tileBits > 14)
                                                    tileBits = 14;
                                                int polyBits = 22 - tileBits;
                                                dtNavMeshParams params;
                                                rcVcopy(params.orig, bmin);
                                                params.tileWidth = cellsize * tileSize;
                                                params.tileHeight = cellsize * tileSize;
                                                params.maxTiles = 1 << tileBits;
                                                params.maxPolys = 1 << polyBits;
                                                dtnaveMesh.init(&params); }))
        // .function("init2d", optional_override([](dtNavMesh &dtnaveMesh, Vec3 min, Vec3 max,const int polyBits)
        //                                      {
        //                                         const float *bmin = min.data();
        //                                         const float *bmax = max.data();
        //                                         dtNavMeshParams params;
        //                                         rcVcopy(params.orig, bmin);
        //                                         params.tileWidth = bmax[0] - bmin[0];
        //                                         params.tileHeight = bmax[2] - bmin[2];
        //                                         params.maxTiles = 1;
        //                                         params.maxPolys = 1 << polyBits;
        //                                         dtnaveMesh.init(&params); }))
        .function("getMaxTiles", &dtNavMesh::getMaxTiles)
        .function("addTile", optional_override([](dtNavMesh &dtnaveMesh, rcConfig m_cfg, TitleConfig titleconfig, val data_ptr, LayaOffMeshConnection *linkMesh, LayaConvexVolume *volumes)
                                               {
                                                    float now = emscripten_get_now();
                                                    int navDataSize = 0;
                                                    unsigned char *navData = creatNavMeshData(m_cfg, titleconfig, data_ptr, navDataSize, linkMesh, volumes);
                                                    printMsg("***creatNavMeshData", &now);
                                                    if (navData)
                                                    {
                                                        dtnaveMesh.addTile(navData, navDataSize, DT_TILE_FREE_DATA, 0, 0);
                                                        printMsg("***addTile", &now);
                                                    } 
                                                }),
                  allow_raw_pointers())
        .function("getTile", select_overload<const dtMeshTile *(int) const>(&dtNavMesh::getTile), allow_raw_pointers())
        .function("getTileRefAt", &dtNavMesh::getTileRefAt)
        .function("getPolyFlags", optional_override([](dtNavMesh &dtnaveMesh, dtTileRef ref)
                                                    {
                                                        unsigned short flags = 0;
                                                        dtnaveMesh.getPolyFlags(ref, &flags);
                                                        return flags; }))
        .function("getPolyArea", optional_override([](dtNavMesh &dtnaveMesh, dtTileRef ref)
                                                   {
                                                        unsigned char resultArea = 0;
                                                        dtnaveMesh.getPolyArea(ref, &resultArea);
                                                        return resultArea; }))
        .function("removeTile", optional_override([](dtNavMesh &dtnaveMesh, const int tx, const int ty)
                                                  { dtnaveMesh.removeTile(dtnaveMesh.getTileRefAt(tx, ty, 0), 0, 0); }))
        .function("getOffMeshConnectionPolyEndPoints", optional_override([](dtNavMesh &dtnaveMesh, dtPolyRef prevRef, dtPolyRef polyRef, val startPos, val endPos)
                                                                         {
                                                                            intptr_t startSize = 0;
                                                                            float *startPosData = GetArray<float>(startPos, &startSize);
                                                                            intptr_t endSize = 0;
                                                                            float *endPosData = GetArray<float>(endPos, &endSize);
                                                                            dtStatus status = dtnaveMesh.getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPosData, endPosData);
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
        .function("getPolyHeight", optional_override([](dtNavMeshQuery &dtnavMeshQuery, dtPolyRef ref, Vec3 vecData)
                                                     {
                                                        float h = 0;
                                                        dtStatus status = dtnavMeshQuery.getPolyHeight(ref, vecData.data(), &h);
                                                        auto out = val::object();
                                                        out.set("height", val(h));
                                                        out.set("Status", val(status));
                                                        return out; }))
        .function("findDistanceToWall", optional_override([](dtNavMeshQuery &dtnavMeshQuery, dtRefPointData refpoint, dtQueryFilter &filter_ptr, const float maxRadius)
                                                          {
                                                            float hitPos[3];
                                                            float hitNormal[3];
                                                            float hitdist;
                                                            const dtQueryFilter *filter = (const dtQueryFilter *)&filter_ptr;
                                                            dtStatus status = dtnavMeshQuery.findDistanceToWall(refpoint.dtpolyRef, refpoint.data, maxRadius, filter, &hitdist, &hitPos[0], &hitNormal[0]);
                                                            auto out = val::object();
                                                            out.set("dist", val(hitdist));
                                                            out.set("pos", CreateArray<float>(&hitPos[0], 3));
                                                            out.set("normal", CreateArray<float>(&hitNormal[0], 3));
                                                            out.set("Status", val(status));
                                                            return out; }))
        .function("findPath", optional_override([](dtNavMeshQuery &dtnavMeshQuery,
                                                   dtRefPointData &startdata, dtRefPointData &enddata, dtQueryFilter &filter_ptr, int MAX_POLYS)
                                                {
                                                    const dtQueryFilter *filter = (const dtQueryFilter *)&filter_ptr;
                                                    dtPolyRef m_polys[MAX_POLYS];
                                                    int pathCount = 0;
                                                    dtStatus status = dtnavMeshQuery.findPath(startdata.dtpolyRef, enddata.dtpolyRef, startdata.data, enddata.data, filter, m_polys, &pathCount, MAX_POLYS);
                                                    auto out = val::object();
                                                    out.set("Status", val(status));
                                                    out.set("polys", CreateArray<dtPolyRef>(&m_polys[0], pathCount));
                                                    return out; }))
        .function("closestPointOnPoly", optional_override([](dtNavMeshQuery &dtnavMeshQuery, dtPolyRef ref, val data)
                                                          {
                                                                const std::vector<float> &vecData = convertJSArrayToNumberVector<float>(data);
                                                                float iterPos[3];
                                                                dtnavMeshQuery.closestPointOnPoly(ref, vecData.data(), iterPos, 0);
                                                                return CreateArray<float>(&iterPos[0], 3); }))
        .function("closestPointOnPolyByRefPointData", optional_override([](dtNavMeshQuery &dtnavMeshQuery, dtRefPointData refdata)
                                                                        {
                                                                            float iterPos[3];
                                                                            dtnavMeshQuery.closestPointOnPoly(refdata.dtpolyRef, refdata.data, iterPos, 0);
                                                                            return CreateArray<float>(&iterPos[0], 3); }),
                  allow_raw_pointers())
        .function("findStraightPath", optional_override([](dtNavMeshQuery &dtnavMeshQuery,
                                                           Vec3 startPos, Vec3 endPos,
                                                           val path, int pathSize, const int maxStraightPath)
                                                        {
                                                                float steerPath[maxStraightPath * 3];
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
                                                           Vec3 startPos, Vec3 endPos,
                                                           dtQueryFilter &filter_ptr, const int maxVisitedSize)
                                                        {
                                                                const dtQueryFilter *filter = (const dtQueryFilter *)&filter_ptr;
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
                                                          Vec3 &center, Vec3 halfExtents, dtQueryFilter &filter_ptr)
                                                       {
                                                                const dtQueryFilter *filter = (const dtQueryFilter *)&filter_ptr;
                                                                dtRefPointData refpoint;
                                                                dtnavMeshQuery.findNearestPoly(center.data(), halfExtents.data(), filter, &refpoint.dtpolyRef, &refpoint.data[0]);
                                                                return refpoint; }))
        .function("findRandomPoint", optional_override([](dtNavMeshQuery &dtnavMeshQuery, dtQueryFilter &filter_ptr, dtRefPointData &randomdata)
                                                       {
                                                                const dtQueryFilter *filter = (const dtQueryFilter *)&filter_ptr;
                                                                return dtnavMeshQuery.findRandomPoint(filter, frand, &randomdata.dtpolyRef, randomdata.data); }))
        .function("findRandomPointAroundCircle", optional_override([](dtNavMeshQuery &dtnavMeshQuery,
                                                                      dtRefPointData &startPoint, float maxRadius,
                                                                      dtQueryFilter &filter_ptr, dtRefPointData &randomdata)
                                                                   {
                                                                    const dtQueryFilter *filter = (const dtQueryFilter *)&filter_ptr;
                                                                    return dtnavMeshQuery.findRandomPointAroundCircle(startPoint.dtpolyRef, startPoint.data, maxRadius, filter, frand, &randomdata.dtpolyRef, randomdata.data); }));
}
