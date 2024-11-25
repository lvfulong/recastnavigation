
#include <stdlib.h>
#include <stdio.h>
#include <cstdint>
#include <cstring>
#include <math.h>
#include "LayaMath.h"

class NavTileData
{
private:
    float *m_triVertex;
    int *m_triIndex;
    uint8_t *m_triFlag;
    int m_triVertexCount;
    int m_triIndexCount;
    int m_triFlagCount;
    friend class NavTileCache;
    friend class NavTileCache2D;

public:
    NavTileData() : m_triVertex(0), m_triIndex(0), m_triFlag(0)
    {
    }

    void setTriVertex(const float *data_ptr, int triVertexCount)
    {
        delete[] m_triVertex;
        if (triVertexCount < 0)
        {
            return;
        }
        m_triVertexCount = triVertexCount;
        m_triVertex = new float[triVertexCount];
        memcpy(m_triVertex, data_ptr, triVertexCount * sizeof(float));
    }

    void setTriIndex(const int *data_ptr, int triIndexCount)
    {
        delete[] m_triIndex;
        if (triIndexCount < 0)
        {
            return;
        }
        m_triIndexCount = triIndexCount;
        m_triIndex = new int[triIndexCount];
        memcpy(m_triIndex, data_ptr, triIndexCount * sizeof(int));
    }

    void setTriFlag(const uint8_t *data_ptr, int triFlagCount)
    {
        delete[] m_triFlag;
        if (triFlagCount < 0)
        {
            return;
        }
        m_triFlagCount = triFlagCount;
        m_triFlag = new uint8_t[triFlagCount];
        memcpy(m_triFlag, data_ptr, triFlagCount * sizeof(uint8_t));
    }
    ~NavTileData()
    {
        dtFree(m_triVertex);
        dtFree(m_triIndex);
        dtFree(m_triFlag);
    }
};

class NavTileCache
{
protected:
    const NavTileData *m_TileData;
    float *m_triVertex;
    uint8_t *m_triFlag;

public:
    NavTileCache() : m_TileData(0), m_triVertex(0), m_triFlag(0) {}
    void init(const NavTileData *data)
    {
        m_TileData = data;
        m_triVertex = (float *)dtAlloc(sizeof(float) * data->m_triVertexCount, DT_ALLOC_PERM);
        memcpy(m_triVertex, data->m_triVertex, data->m_triVertexCount * sizeof(float));
        // m_triFlag = new uint8_t[data->m_triFlagCount];
        m_triFlag = (uint8_t *)dtAlloc(sizeof(uint8_t) * data->m_triFlagCount, DT_ALLOC_PERM);
        memcpy(m_triFlag, data->m_triFlag, data->m_triFlagCount * sizeof(uint8_t));
    }

    void transfromData(const float *elem)
    {
        int vertexCount = m_TileData->m_triVertexCount / 3;
        for (int i = 0; i < vertexCount; i++)
        {
            int triIndex = i * 3;
            transformCoordinate(elem, &m_TileData->m_triVertex[triIndex], &m_triVertex[triIndex]);
        }
    }

    void setFlag(uint8_t flag)
    {
        for (int i = 0; i < m_TileData->m_triFlagCount; i++)
        {
            m_triFlag[i] = flag;
        }
    }

    int getTriVertexCount() { return m_TileData->m_triVertexCount; }
    int getTriIndexCount() { return m_TileData->m_triIndexCount; }
    int getTriFlagCount() { return m_TileData->m_triFlagCount; }
    const float *getVerts() { return m_triVertex; }
    const int *getTris() { return m_TileData->m_triIndex; }
    const uint8_t *getFlags() { return m_triFlag; }
    ~NavTileCache()
    {
        if (m_triVertex)
        {
            dtFree(m_triVertex);
        }

        if (m_triFlag)
        {
            dtFree(m_triFlag);
        }
    }
};
