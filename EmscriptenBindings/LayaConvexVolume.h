#include "Recast.h"
#include "RecastAssert.h"

// Returns true if 'c' is left of line 'a'-'b'.
inline bool left(const float *a, const float *b, const float *c)
{
    const float u1 = b[0] - a[0];
    const float v1 = b[2] - a[2];
    const float u2 = c[0] - a[0];
    const float v2 = c[2] - a[2];
    return u1 * v2 - v1 * u2 < 0;
}

// Returns true if 'a' is more lower-left than 'b'.
inline bool cmppt(const float *a, const float *b)
{
    if (a[0] < b[0])
        return true;
    if (a[0] > b[0])
        return false;
    if (a[2] < b[2])
        return true;
    if (a[2] > b[2])
        return false;
    return false;
}
// Calculates convex hull on xz-plane of points on 'pts',
// stores the indices of the resulting hull in 'out' and
// returns number of points on hull.
static int convexhull(const float *pts, int npts, int *out)
{
    // Find lower-leftmost point.
    int hull = 0;
    for (int i = 1; i < npts; ++i)
        if (cmppt(&pts[i * 3], &pts[hull * 3]))
            hull = i;
    // Gift wrap hull.
    int endpt = 0;
    int i = 0;
    do
    {
        out[i++] = hull;
        endpt = 0;
        for (int j = 1; j < npts; ++j)
            if (hull == endpt || left(&pts[hull * 3], &pts[endpt * 3], &pts[j * 3]))
                endpt = j;
        hull = endpt;
    } while (endpt != out[0]);

    return i;
}

class LayaConvexVolume
{
public:
    static const int MAX_CONVEX_COUNT = 256;
    float m_verts[MAX_CONVEX_COUNT * 3 * 2];
    float m_transfroms[MAX_CONVEX_COUNT * 16];
    unsigned char m_area[MAX_CONVEX_COUNT];
    int m_ConvexCount;

    inline int addCovexVoume(const float *datas, const float *trans, const unsigned char area)
    {
        if (m_ConvexCount >= MAX_CONVEX_COUNT)
            return -1;
        updateCovexVoume(m_ConvexCount, datas, trans, area);
        m_ConvexCount++;
        return m_ConvexCount - 1;
    }

    inline void updateCovexVoume(int id, const float *datas, const float *trans, const unsigned char area)
    {
        if (id >= MAX_CONVEX_COUNT)
            return;
        memcpy(&m_verts[id * 6], datas, 6 * sizeof(float));
        memcpy(&m_transfroms[id * 16], trans, 16 * sizeof(float));
        m_area[id] = area;
    }

    inline void deleteCovexVoume(int i)
    {
        m_ConvexCount--;
        memcpy(&m_verts[i * 6], &m_verts[m_ConvexCount * 6], 6 * sizeof(float));
        memcpy(&m_transfroms[i * 16], &m_transfroms[m_ConvexCount * 6], 16 * sizeof(float));
        m_area[i] = m_area[m_ConvexCount];
    }
};

bool pointInBound(const float *p)
{
    if (rcAbs(p[0]) >= 1.0f)
        return false;
    if (rcAbs(p[1]) >= 1.0f)
        return false;
    if (rcAbs(p[2]) >= 1.0f)
        return false;
    return true;
}

void rcMarkConvexVolume(rcContext *ctx, const LayaConvexVolume *volume, const int index, rcCompactHeightfield &chf)
{
    rcAssert(ctx);

    rcScopedTimer timer(ctx, RC_TIMER_MARK_CONVEXPOLY_AREA);
    const float *verts = &volume->m_verts[index * 6];
    const float *trans = &volume->m_transfroms[index * 16];
    const unsigned char areaId = volume->m_area[index];
    float bmin[3], bmax[3];
    rcVcopy(bmin, verts);
    rcVcopy(bmax, &verts[3]);

    int minx = (int)((bmin[0] - chf.bmin[0]) / chf.cs);
    int miny = (int)((bmin[1] - chf.bmin[1]) / chf.ch);
    int minz = (int)((bmin[2] - chf.bmin[2]) / chf.cs);
    int maxx = (int)((bmax[0] - chf.bmin[0]) / chf.cs);
    int maxy = (int)((bmax[1] - chf.bmin[1]) / chf.ch);
    int maxz = (int)((bmax[2] - chf.bmin[2]) / chf.cs);

    if (maxx < 0)
        return;
    if (minx >= chf.width)
        return;
    if (maxz < 0)
        return;
    if (minz >= chf.height)
        return;

    if (minx < 0)
        minx = 0;
    if (maxx >= chf.width)
        maxx = chf.width - 1;
    if (minz < 0)
        minz = 0;
    if (maxz >= chf.height)
        maxz = chf.height - 1;

    // TODO: Optimize.
    for (int z = minz; z <= maxz; ++z)
    {
        for (int x = minx; x <= maxx; ++x)
        {
            const rcCompactCell &c = chf.cells[x + z * chf.width];
            for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
            {
                rcCompactSpan &s = chf.spans[i];
                if (chf.areas[i] == RC_NULL_AREA)
                    continue;
                if ((int)s.y >= miny && (int)s.y <= maxy)
                {
                    float p[3];
                    p[0] = chf.bmin[0] + (x + 0.5f) * chf.cs;
                    p[1] = 0;
                    p[2] = chf.bmin[2] + (z + 0.5f) * chf.cs;
                    transformCoordinate(trans, &p[0], &p[0]);

                    if (pointInBound(&p[0]))
                    {
                        chf.areas[i] = areaId;
                    }
                }
            }
        }
    }
}
