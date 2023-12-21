#include "Recast.h"

class LayaOffMeshConnection
{
public:
    static const int MAX_OFFMESH_CONNECTIONS = 256;
    float m_offMeshConVerts[MAX_OFFMESH_CONNECTIONS * 3 * 2];
    float m_offMeshConRads[MAX_OFFMESH_CONNECTIONS];
    unsigned char m_offMeshConDirs[MAX_OFFMESH_CONNECTIONS];
    unsigned char m_offMeshConAreas[MAX_OFFMESH_CONNECTIONS];
    unsigned short m_offMeshConFlags[MAX_OFFMESH_CONNECTIONS];
    unsigned int m_offMeshConId[MAX_OFFMESH_CONNECTIONS];
    int m_offMeshConCount;

    inline void addOffMeshConnection(std::array<float, 3> spos, std::array<float, 3> epos, const float rad,
                                     unsigned char bidir, unsigned char area, unsigned short flags, unsigned int id)
    {
        if (m_offMeshConCount >= MAX_OFFMESH_CONNECTIONS)
            return;
        float *v = &m_offMeshConVerts[m_offMeshConCount * 3 * 2];
        m_offMeshConRads[m_offMeshConCount] = rad;
        m_offMeshConDirs[m_offMeshConCount] = bidir;
        m_offMeshConAreas[m_offMeshConCount] = area;
        m_offMeshConFlags[m_offMeshConCount] = flags;
        m_offMeshConId[m_offMeshConCount] = id;
        rcVcopy(&v[0], spos.data());
        rcVcopy(&v[3], epos.data());
        m_offMeshConCount++;
    }

    inline void deleteOffMeshConnection(int i)
    {
        m_offMeshConCount--;
        float *src = &m_offMeshConVerts[m_offMeshConCount * 3 * 2];
        float *dst = &m_offMeshConVerts[i * 3 * 2];
        rcVcopy(&dst[0], &src[0]);
        rcVcopy(&dst[3], &src[3]);
        m_offMeshConRads[i] = m_offMeshConRads[m_offMeshConCount];
        m_offMeshConDirs[i] = m_offMeshConDirs[m_offMeshConCount];
        m_offMeshConAreas[i] = m_offMeshConAreas[m_offMeshConCount];
        m_offMeshConFlags[i] = m_offMeshConFlags[m_offMeshConCount];
    }
};
