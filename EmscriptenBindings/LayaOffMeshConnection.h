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

    int getOffMeshConnectionIndexById(int id){
        for(int i=0;i<m_offMeshConCount;i++){
            if(m_offMeshConId[i] == id){
                return i;
            }
        }
        return -1;
    }

    void addOffMeshConnection( unsigned int id,std::array<float, 3> spos, std::array<float, 3> epos, const float rad, unsigned char bidir, unsigned char area)
    {
        int index = this->getOffMeshConnectionIndexById(id);
        if(index<0){
            if (m_offMeshConCount >= MAX_OFFMESH_CONNECTIONS) return;
            index = m_offMeshConCount;
            m_offMeshConCount ++;
        }
        m_offMeshConRads[index] = rad;
        m_offMeshConDirs[index] = bidir;
        m_offMeshConAreas[index] = area;
        m_offMeshConFlags[index] =  1<<int(area-1);
        m_offMeshConId[index] = id;
        float *v = &m_offMeshConVerts[index * 3 * 2];
        rcVcopy(&v[0], spos.data());
        rcVcopy(&v[3], epos.data());
    }

    void deleteOffMeshConnection(int id)
    {
        int idIndex = this->getOffMeshConnectionIndexById(id);
        m_offMeshConCount--;
        if(idIndex == -1|| m_offMeshConCount == idIndex){
            return;
        }
        float *src = &m_offMeshConVerts[m_offMeshConCount * 3 * 2];
        float *dst = &m_offMeshConVerts[idIndex * 3 * 2];
        rcVcopy(&dst[0], &src[0]);
        rcVcopy(&dst[3], &src[3]);
        m_offMeshConRads[idIndex] = m_offMeshConRads[m_offMeshConCount];
        m_offMeshConDirs[idIndex] = m_offMeshConDirs[m_offMeshConCount];
        m_offMeshConAreas[idIndex] = m_offMeshConAreas[m_offMeshConCount];
        m_offMeshConFlags[idIndex] = m_offMeshConFlags[m_offMeshConCount];
    }
};
