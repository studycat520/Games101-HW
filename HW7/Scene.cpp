//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // 1. 判断光线是否与场景相交
    Intersection inter = intersect(ray);
    if (!inter.happened) {
        return Vector3f(0, 0, 0);
    }

    // 2. 处理自发光 (Emission)
    // 如果光线直接击中光源，且是第一次射出（depth=0），则返回光源颜色
    // 如果是反弹光线击中光源（depth>0），我们在直接光照部分已经采样过了，
    // 为了避免双重计数（Double Counting），这里通常返回 0。
    // 但根据伪代码 "If ray r hit a non-emitting object at q"，说明间接光照不计算光源。
    if (inter.m->hasEmission()) {
        if (depth == 0) {
            return inter.m->getEmission();
        } else {
            return Vector3f(0, 0, 0);
        }
    }

    // 准备基本数据
    Vector3f p = inter.coords;      // 交点坐标
    Vector3f N = inter.normal;      // 法线
    Vector3f wo = -ray.direction;   // 观察方向 (指向外部)
    Vector3f L_dir(0), L_indir(0);

    // ============================================
    // Part 1: 直接光照 (Sample Light)
    // ============================================
    Intersection light_inter;
    float light_pdf = 0.0f;
    
    // 在光源上采样一个点 x
    sampleLight(light_inter, light_pdf);
    
    Vector3f x = light_inter.coords;
    Vector3f emit = light_inter.emit;
    Vector3f ws = (x - p).normalized(); // 从点 p 指向光源点 x 的方向
    Vector3f NN = light_inter.normal;   // 光源上的法线
    float dist = (x - p).norm();
    float dist2 = dist * dist;

    // 检查光源是否被遮挡
    // 发射一条从 p 到 x 的光线。注意为了防止自相交，起点 p 需要往法线方向移动一点点 (EPSILON)
    Ray light_to_obj_ray(p, ws);
    Intersection shadow_inter = intersect(light_to_obj_ray);

    // 如果光线击中了物体，且击中的距离非常接近 p 到 x 的距离，说明没被遮挡
    // 这里的 distance - dist > -EPSILON 是为了处理浮点误差
    if (shadow_inter.happened && (shadow_inter.distance - dist > -EPSILON)) {
        // 渲染方程核心公式: L = emit * f_r * cos_theta * cos_theta_x / dist^2 / pdf
        Vector3f f_r = inter.m->eval(ws, wo, N);
        float cos_theta = std::max(0.0f, dotProduct(ws, N));
        float cos_theta_x = std::max(0.0f, dotProduct(-ws, NN));
        
        L_dir = emit * f_r * cos_theta * cos_theta_x / dist2 / light_pdf;
    }

    // ============================================
    // Part 2: 间接光照 (Sample Material & Russian Roulette)
    // ============================================
    
    // 俄罗斯轮盘赌测试
    if (get_random_float() < RussianRoulette) {
        // 采样下一个方向 wi
        Vector3f wi = inter.m->sample(wo, N).normalized();
        float pdf = inter.m->pdf(wo, wi, N);

        // 如果 pdf 太小，数值不稳定，直接忽略
        if (pdf > EPSILON) {
            // 递归追踪
            Ray bounce_ray(p, wi);
            Intersection bounce_inter = intersect(bounce_ray);

            // 如果打到了物体，且物体不发光 (光源已经在 Part 1 处理了)
            if (bounce_inter.happened && !bounce_inter.m->hasEmission()) {
                Vector3f f_r = inter.m->eval(wi, wo, N);
                float cos_theta = std::max(0.0f, dotProduct(wi, N));
                Vector3f L_shade = castRay(bounce_ray, depth + 1);
                
                // 间接光照公式: L = shade(q, wi) * f_r * cos_theta / pdf / P_RR
                L_indir = L_shade * f_r * cos_theta / pdf / RussianRoulette;
            }
        }
    }

    return L_dir + L_indir;
}