//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

class Bounds3
{
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    Vector3f Diagonal() const { return pMax - pMin; }
    int maxExtent() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }
    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f& p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f& p, const Bounds3& b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }
    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
                           const std::array<int, 3>& dirisNeg) const;
};



inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{
    // t_enter: 光线进入所有三个轴 slab 的最晚时间 (max of mins)
    // t_exit:  光线离开所有三个轴 slab 的最早时间 (min of maxs)
    
    // 初始化 t_enter 和 t_exit
    // 注意：这里没有传入 ray 的 tMin 和 tMax，通常我们假设 t_enter 初始为极小，t_exit 初始为极大
    // 但更严谨的做法是结合光线本身的有效范围，这里我们简化逻辑
    float t_enter = -std::numeric_limits<float>::infinity();
    float t_exit = std::numeric_limits<float>::infinity();

    for (int i = 0; i < 3; i++)
    {
        // 计算光线与当前轴 (i) 的两个平面的交点 t 值
        // pMin 和 pMax 是包围盒的最小/最大顶点
        // 利用预计算的 invDir (1/dir) 避免除法，提高速度
        float t_min = (pMin[i] - ray.origin[i]) * invDir[i];
        float t_max = (pMax[i] - ray.origin[i]) * invDir[i];

        // 如果光线在该轴方向是负的，说明我们先碰到的是 pMax 面，后碰到 pMin 面
        // 所以需要交换 t_min 和 t_max
        if (dirIsNeg[i]) // 或者用 if (invDir[i] < 0)
        {
            std::swap(t_min, t_max);
        }

        // 更新整体的进入时间和离开时间
        // 我们必须“晚点进，早点出”才能保证在盒子里
        t_enter = std::max(t_enter, t_min);
        t_exit = std::min(t_exit, t_max);
    }

    // 判断相交条件：
    // 1. t_enter <= t_exit: 进入时间必须早于或等于离开时间（否则就是没进盒子就出了，即错过了）
    // 2. t_exit >= 0: 离开时间必须是正的（盒子不能全在光线背面）
    return t_enter <= t_exit && t_exit >= 0;
    
}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
