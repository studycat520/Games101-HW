#pragma once

#include "Object.hpp"

#include <cstring>


bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, const Vector3f& orig,
                          const Vector3f& dir, float& tnear, float& u, float& v)
{
    // 1. 准备边向量 E1, E2
    Vector3f E1 = v1 - v0;
    Vector3f E2 = v2 - v0;

    // 2. 准备辅助向量 P (P = dir x E2)
    Vector3f S = orig - v0;
    Vector3f S1 = crossProduct(dir, E2);
    Vector3f S2 = crossProduct(S, E1);

    // 3. 计算分母 (det)
    // 根据公式: det = (dir x E2) . E1 = S1 . E1
    float det = dotProduct(S1, E1);
    float invDet = 1.0f / det; // 提前计算倒数，后面乘法比除法快

    // 4. 计算 t, u, v
    // 这里的顺序很重要，按照 Möller-Trumbore 公式推导
    
    // 计算 t_raw (距离)
    float t_raw = dotProduct(S2, E2) * invDet;
    
    // 计算 u (重心坐标 1)
    float u_raw = dotProduct(S1, S) * invDet;
    
    // 计算 v (重心坐标 2)
    float v_raw = dotProduct(S2, dir) * invDet;


    // 5. 判定交点是否有效 (Validity Check)
    // 条件1: t 必须大于 0 (交点在光线前方)
    // 条件2: u 必须在 [0, 1] 之间，且 v 必须在 [0, 1] 之间，且 u + v <= 1
    // 注意: 由于浮点数精度问题，有时候会判定 >= 0
    if (t_raw > 0 && u_raw >= 0 && v_raw >= 0 && (u_raw + v_raw) <= 1)
    {
        // 只有当所有条件都满足时，才更新输出参数并返回 true
        tnear = t_raw;
        u = u_raw;
        v = v_raw;
        return true;
    }

    return false;
}

class MeshTriangle : public Object
{
public:
    MeshTriangle(const Vector3f* verts, const uint32_t* vertsIndex, const uint32_t& numTris, const Vector2f* st)
    {
        uint32_t maxIndex = 0;
        for (uint32_t i = 0; i < numTris * 3; ++i)
            if (vertsIndex[i] > maxIndex)
                maxIndex = vertsIndex[i];
        maxIndex += 1;
        vertices = std::unique_ptr<Vector3f[]>(new Vector3f[maxIndex]);
        memcpy(vertices.get(), verts, sizeof(Vector3f) * maxIndex);
        vertexIndex = std::unique_ptr<uint32_t[]>(new uint32_t[numTris * 3]);
        memcpy(vertexIndex.get(), vertsIndex, sizeof(uint32_t) * numTris * 3);
        numTriangles = numTris;
        stCoordinates = std::unique_ptr<Vector2f[]>(new Vector2f[maxIndex]);
        memcpy(stCoordinates.get(), st, sizeof(Vector2f) * maxIndex);
    }

    bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t& index,
                   Vector2f& uv) const override
    {
        bool intersect = false;
        for (uint32_t k = 0; k < numTriangles; ++k)
        {
            const Vector3f& v0 = vertices[vertexIndex[k * 3]];
            const Vector3f& v1 = vertices[vertexIndex[k * 3 + 1]];
            const Vector3f& v2 = vertices[vertexIndex[k * 3 + 2]];
            float t, u, v;
            if (rayTriangleIntersect(v0, v1, v2, orig, dir, t, u, v) && t < tnear)
            {
                tnear = t;
                uv.x = u;
                uv.y = v;
                index = k;
                intersect |= true;
            }
        }

        return intersect;
    }

    void getSurfaceProperties(const Vector3f&, const Vector3f&, const uint32_t& index, const Vector2f& uv, Vector3f& N,
                              Vector2f& st) const override
    {
        const Vector3f& v0 = vertices[vertexIndex[index * 3]];
        const Vector3f& v1 = vertices[vertexIndex[index * 3 + 1]];
        const Vector3f& v2 = vertices[vertexIndex[index * 3 + 2]];
        Vector3f e0 = normalize(v1 - v0);
        Vector3f e1 = normalize(v2 - v1);
        N = normalize(crossProduct(e0, e1));
        const Vector2f& st0 = stCoordinates[vertexIndex[index * 3]];
        const Vector2f& st1 = stCoordinates[vertexIndex[index * 3 + 1]];
        const Vector2f& st2 = stCoordinates[vertexIndex[index * 3 + 2]];
        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
    }

    Vector3f evalDiffuseColor(const Vector2f& st) const override
    {
        float scale = 5;
        float pattern = (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
        return lerp(Vector3f(0.815, 0.235, 0.031), Vector3f(0.937, 0.937, 0.231), pattern);
    }

    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;
};
