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
    // TO DO Implement Path Tracing Algorithm here
    Vector3f L_dir = {0, 0, 0};
    Vector3f L_indir = {0, 0, 0};
	float pdf_light, pdf_object;
	Intersection inter, lightInter;
	inter = intersect(ray);
	if (!inter.happened)
		return {};
	if (inter.m->hasEmission()) 
		return inter.m->getEmission();
	sampleLight(lightInter, pdf_light);
	Vector3f orig, dir,v;
	v = lightInter.coords - inter.coords;
	orig = inter.coords;
	dir = v.normalized();
	Ray Object2ray = {orig, dir};
	Intersection block = intersect(Object2ray);
	float distance2 = v.x * v.x + v.y * v.y + v.z * v.z;
	//lightInter.distance can't use because it hasn't been initialize
	if (block.distance - v.norm() > -EPSILON) {
		L_dir = lightInter.emit * inter.m->eval(ray.direction, dir, inter.normal) * dotProduct(dir, inter.normal) * dotProduct(-dir, lightInter.normal) / pdf_light / distance2;
	}
	if (get_random_float() > RussianRoulette) {
		return L_dir;
	}

	Vector3f nextobjdir = inter.m->sample(ray.direction, inter.normal).normalized();
	Ray obj2next = {inter.coords, nextobjdir};
	Intersection nextObject = intersect(obj2next);
	if (nextObject.happened && !nextObject.m->hasEmission()) {
		pdf_object = inter.m->pdf(ray.direction, nextobjdir, inter.normal);
		L_indir = castRay(obj2next, depth + 1) * inter.m->eval(ray.direction, nextobjdir, inter.normal) * dotProduct(nextobjdir, inter.normal) / pdf_object / RussianRoulette;
	}
	return L_dir + L_indir;
}