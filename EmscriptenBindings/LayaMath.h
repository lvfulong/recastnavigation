
inline void transformCoordinate(const float *elem, const float *p, float *out)
{
    float px = p[0];
    float py = p[1];
    float pz = p[2];
    float w = px * elem[3] + py * elem[7] + pz * elem[11] + elem[15];
    out[0] = (px * elem[0] + py * elem[4] + pz * elem[8] + elem[12]) / w;
    out[1] = (px * elem[1] + py * elem[5] + pz * elem[9] + elem[13]) / w;
    out[2] = (px * elem[2] + py * elem[6] + pz * elem[10] + elem[14]) / w;
}