#include "LiteMath/LiteMath.h"
#include "LiteMath/Image2d.h"

// stb_image is a single-header C library, which means one of your cpp files must have
//    #define STB_IMAGE_IMPLEMENTATION
//    #define STB_IMAGE_WRITE_IMPLEMENTATION
// since Image2d already defines the implementation, we don't need to do that here.
#include "stb_image.h"
#include "stb_image_write.h"

#include <SDL_keycode.h>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <SDL.h>

#include "mesh.h"
using namespace cmesh4;

using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float4;
using LiteMath::int2;
using LiteMath::int3;
using LiteMath::int4;
using LiteMath::uint2;
using LiteMath::uint3;
using LiteMath::uint4;


struct SdfGrid
{
  uint3 size;
  std::vector<float> data;
};

void save_sdf_grid(const SdfGrid &scene, const std::string &path)
{
  std::ofstream fs(path, std::ios::binary);
  fs.write((const char *)&scene.size, 3 * sizeof(unsigned));
  fs.write((const char *)scene.data.data(), scene.size.x * scene.size.y * scene.size.z * sizeof(float));
  fs.flush();
  fs.close();
}

void load_sdf_grid(SdfGrid &scene, const std::string &path)
{
  std::ifstream fs(path, std::ios::binary);
  fs.read((char *)&scene.size, 3 * sizeof(unsigned));
  scene.data.resize(scene.size.x * scene.size.y * scene.size.z);
  fs.read((char *)scene.data.data(), scene.size.x * scene.size.y * scene.size.z * sizeof(float));
  fs.close();
}

struct SdfOctreeNode
{
  float values[8];
  unsigned offset;
};

struct SdfOctree
{
  std::vector<SdfOctreeNode> nodes;
};

void save_sdf_octree(const SdfOctree &scene, const std::string &path)
{
  std::ofstream fs(path, std::ios::binary);
  size_t size = scene.nodes.size();
  fs.write((const char *)&size, sizeof(unsigned));
  fs.write((const char *)scene.nodes.data(), size * sizeof(SdfOctreeNode));
  fs.flush();
  fs.close();
}

void load_sdf_octree(SdfOctree &scene, const std::string &path)
{
  std::ifstream fs(path, std::ios::binary);
  unsigned sz = 0;
  fs.read((char *)&sz, sizeof(unsigned));
  scene.nodes.resize(sz);
  fs.read((char *)scene.nodes.data(), scene.nodes.size() * sizeof(SdfOctreeNode));
  fs.close();
}

struct AppData
{
  int width;
  int height;
  SdfGrid loaded_grid;
  int z_level = 32;
};

void draw_sdf_grid_slice(const SdfGrid &grid, int z_level, int voxel_size,
                         int width, int height, std::vector<uint32_t> &pixels)
{
  constexpr uint32_t COLOR_EMPTY = 0xFF333333;
  constexpr uint32_t COLOR_FULL = 0xFFFFA500;
  constexpr uint32_t COLOR_BORDER = 0xFF000000;

  for (int y = 0; y < grid.size.y; y++)
  {
    for (int x = 0; x < grid.size.x; x++)
    {
      int index = x + y * grid.size.x + z_level * grid.size.x * grid.size.y;
      uint32_t color = grid.data[index] < 0 ? COLOR_FULL : COLOR_EMPTY;
      for (int i = 0; i <= voxel_size; i++)
      {
        for (int j = 0; j <= voxel_size; j++)
        {
          int pixel_idx = (x * voxel_size + i) + ((height - 1) - (y * voxel_size + j)) * width;
          if (i == 0 || i == voxel_size || j == 0 || j == voxel_size)
            pixels[pixel_idx] = COLOR_BORDER;
          else
            pixels[pixel_idx] = color;
        }
      }
    }
  }
}

void save_frame(const char* filename, const std::vector<uint32_t>& frame, uint32_t width, uint32_t height)
{
  LiteImage::Image2D<uint32_t> image(width, height, frame.data());

  
  for (uint32_t i = 0; i < width * height; i++) {
    uint32_t& pixel = image.data()[i];
    auto a = (pixel & 0xFF000000);
    auto r = (pixel & 0x00FF0000) >> 16;
    auto g = (pixel & 0x0000FF00);      // Conv from ARGB to ABGR
    auto b = (pixel & 0x000000FF) << 16;
    pixel = a | b | g | r;
  }

  if (LiteImage::SaveImage(filename, image))
    std::cout << "Image saved to " << filename << std::endl;
  else
    std::cout << "Image could not be saved to " << filename << std::endl;
}



// my code 

#include <vector>
#include <memory>
#include <limits>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Определение структур

struct CameraData {
    float3 position;
    float yaw;      // угол по горизонтали
    float pitch;   // угол по вертикали
    float3 up;
    float fov_y;
    float aspect_ratio;
};

struct Ray {
    float3 origin;
    float3 direction;
    Ray(){}
    Ray(LiteMath::float3 o, LiteMath::float3 d) : origin(o), direction(d) {}
};

struct Material {
    float3 diffuse_color;
    bool is_mirror = false;
};

struct Intersection {
    float t;
    float3 position;
    float3 normal;
    Material material;
    
};

struct Light {
    float3 position;
    float intensity;
};

class SceneObject {
public:
    virtual bool intersect(const Ray& ray, Intersection& isect) const = 0;
    Material material;
};

// Класс плоскости (A1)
class Plane : public SceneObject {
public:
    float a, b, c, d;
    float3 normal;

    Plane(float a, float b, float c, float d) : a(a), b(b), c(c), d(d) {
        normal = normalize(float3(a, b, c));
    }

    bool intersect(const Ray& ray, Intersection& isect) const override {
        float denom = dot(ray.direction, normal);
        if (std::abs(denom) < 1e-6f) {
            return false;     // Луч параллелен плоскости
        }
        float t = -(dot(ray.origin, normal) + d) / denom;
        if (t < 0) {
            return false;    // Пересечение за началом луча
        }
        isect.t = t;
        isect.position = ray.origin + t * ray.direction;
        isect.normal = normal;
        isect.material = material;
        return true;
    }
};

// Структура для AABB
struct AABB {
    float3 min;
    float3 max;

    AABB() : min(float3(std::numeric_limits<float>::max())), max(float3(-std::numeric_limits<float>::max())) {}
    
    void expand(const float3& point) {
        min = LiteMath::min(min, point);
        max = LiteMath::max(max, point);
    }

    bool intersect(const Ray& ray, float t_min, float t_max) const {
        for (int axis = 0; axis < 3; axis++) {
            float invD = 1.0f / ray.direction[axis];
            float t0 = (min[axis] - ray.origin[axis]) * invD;
            float t1 = (max[axis] - ray.origin[axis]) * invD;
            if (invD < 0.0f) std::swap(t0, t1);
            t_min = std::max(t0, t_min);
            t_max = std::min(t1, t_max);
            if (t_max <= t_min) return false;
        }
        return true;
    }
};

// Узел BVH (для A3)
struct BVHNode {
    AABB bounds;
    std::vector<int> triangle_indices;
    std::unique_ptr<BVHNode> left;
    std::unique_ptr<BVHNode> right;

    bool is_leaf() const { return !left && !right; }
};

// Класс для меша (A2 и A3)
class Mesh : public SceneObject {
public:
    std::vector<float3> vertices;
    std::vector<int3> indices;
    std::vector<float3> triangle_normals;
    std::unique_ptr<BVHNode> bvh_root;
    bool use_bvh = false; //  A2 (наивный) и A3 (BVH)

  void load_from_obj(const std::string& path) {
    cmesh4::SimpleMesh loaded_mesh = cmesh4::LoadMeshFromObj(path.c_str(), true);
    vertices.clear();
    indices.clear();

    float3 min_bound = float3(std::numeric_limits<float>::max());
    float3 max_bound = float3(std::numeric_limits<float>::lowest());
    for (const auto& v : loaded_mesh.vPos4f) {
        float3 pos = float3(v.x, v.y, v.z);
        min_bound = LiteMath::min(min_bound, pos);
        max_bound = LiteMath::max(max_bound, pos);
    }
    float3 center = (min_bound + max_bound) * 0.5f;
    float3 size = max_bound - min_bound;
    float max_size = std::max({size.x, size.y, size.z});

    float scale = 1.0f / max_size;
    for (const auto& v : loaded_mesh.vPos4f) {
        float3 pos = float3(v.x, v.y, v.z);
        pos = (pos - center) * scale;
        vertices.push_back(pos);
    }

    for (size_t i = 0; i < loaded_mesh.indices.size(); i += 3) {
        indices.push_back(int3(loaded_mesh.indices[i], 
                               loaded_mesh.indices[i + 1], 
                               loaded_mesh.indices[i + 2]));
    }

    compute_normals();
  }

    void compute_normals() {
      triangle_normals.clear();
      triangle_normals.resize(indices.size(), float3(0, 0, 0));

      for (size_t i = 0; i < indices.size(); ++i) {
          const int3& idx = indices[i];
          float3 v0 = vertices[idx.x];
          float3 v1 = vertices[idx.y];
          float3 v2 = vertices[idx.z];

          float3 edge1 = v1 - v0;
          float3 edge2 = v2 - v0;
          float3 normal = cross(edge1, edge2);

          float len = length(normal);
          if (len > 0.0f) {
              normal /= len;
          }
          triangle_normals[i] = normal;
      }
    }

    void build_bvh() {
        std::vector<int> tri_indices(indices.size());
        for (size_t i = 0; i < indices.size(); i++) tri_indices[i] = i;
        bvh_root = build_bvh_recursive(tri_indices, 0, tri_indices.size());
        use_bvh = true;
    }

    bool intersect(const Ray& ray, Intersection& isect) const override {
        float t_min = std::numeric_limits<float>::max();
        if (use_bvh && bvh_root) {
            return intersect_bvh(bvh_root.get(), ray, isect, t_min);
        } else {
            return intersect_naive(ray, isect, t_min);
        }
    }

private:
    bool ray_triangle_intersect(const Ray& ray, const float3& v0, const float3& v1, const float3& v2, float& t, float& u, float& v) const {
        const float EPSILON = 1e-6f;
        float3 edge1 = v1 - v0;
        float3 edge2 = v2 - v0;
        float3 h = cross(ray.direction, edge2);
        float a = dot(edge1, h);
        if (a > -EPSILON && a < EPSILON) return false;
        float f = 1.0f / a;
        float3 s = ray.origin - v0;
        u = f * dot(s, h);
        if (u < 0.0f || u > 1.0f) return false;
        float3 q = cross(s, edge1);
        v = f * dot(ray.direction, q);
        if (v < 0.0f || u + v > 1.0f) return false;
        t = f * dot(edge2, q);
        return t > EPSILON;
    }

    bool intersect_naive(const Ray& ray, Intersection& isect, float& t_min) const {
        bool hit = false;
        for (size_t i = 0; i < indices.size(); i++) {
            const int3& idx = indices[i];
            float3 v0 = vertices[idx.x];
            float3 v1 = vertices[idx.y];
            float3 v2 = vertices[idx.z];
            float t, u, v;
            if (ray_triangle_intersect(ray, v0, v1, v2, t, u, v) && t < t_min) {
                t_min = t;
                isect.t = t;
                isect.position = ray.origin + t * ray.direction;
                isect.normal = triangle_normals[i];
                isect.material = material;
                hit = true;
            }
        }
        return hit;
    }

    std::unique_ptr<BVHNode> build_bvh_recursive(std::vector<int>& tri_indices, size_t start, size_t end) {
        auto node = std::make_unique<BVHNode>();
        if (end - start <= 4)
        {
            node->triangle_indices.assign(tri_indices.begin() + start, tri_indices.begin() + end);
            for (int idx : node->triangle_indices) {
                const int3& tri = indices[idx];
                node->bounds.expand(vertices[tri.x]);
                node->bounds.expand(vertices[tri.y]);
                node->bounds.expand(vertices[tri.z]);
            }
            return node;
        }

              //AABB для всех треугольников
        AABB centroid_bounds;
        std::vector<float3> centroids(end - start);
        for (size_t i = start; i < end; i++) {
            const int3& tri = indices[tri_indices[i]];
            float3 centroid = (vertices[tri.x] + vertices[tri.y] + vertices[tri.z]) / 3.0f;
            centroids[i - start] = centroid;
            centroid_bounds.expand(centroid);
        }

        float3 extent = centroid_bounds.max - centroid_bounds.min;
        int axis = (extent.x > extent.y && extent.x > extent.z) ? 0 : (extent.y > extent.z ? 1 : 2);

        std::sort(tri_indices.begin() + start, tri_indices.begin() + end,
                  [&](int a, int b) {
                      float3 ca = (vertices[indices[a].x] + vertices[indices[a].y] + vertices[indices[a].z]) / 3.0f;
                      float3 cb = (vertices[indices[b].x] + vertices[indices[b].y] + vertices[indices[b].z]) / 3.0f;
                      return ca[axis] < cb[axis];
                  });

        size_t mid = start + (end - start) / 2;
        node->left = build_bvh_recursive(tri_indices, start, mid);
        node->right = build_bvh_recursive(tri_indices, mid, end);
        node->bounds = node->left->bounds;
        node->bounds.expand(node->right->bounds.min);
        node->bounds.expand(node->right->bounds.max);
        return node;
    }

    bool intersect_bvh(const BVHNode* node, const Ray& ray, Intersection& isect, float& t_min) const {
        if (!node->bounds.intersect(ray, 0.0f, t_min)) return false;
        if (node->is_leaf()) {
            bool hit = false;
            for (int idx : node->triangle_indices) {
                const int3& tri = indices[idx];
                float t, u, v;
                if (ray_triangle_intersect(ray, vertices[tri.x], vertices[tri.y], vertices[tri.z], t, u, v) && t < t_min) {
                    t_min = t;
                    isect.t = t;
                    isect.position = ray.origin + t * ray.direction;
                    isect.normal = triangle_normals[idx];
                    isect.material = material;
                    hit = true;
                }
            }
            return hit;
        }
        bool hit_left = intersect_bvh(node->left.get(), ray, isect, t_min);
        bool hit_right = intersect_bvh(node->right.get(), ray, isect, t_min);
        return hit_left || hit_right;
    }
};


class Camera {
public:
    float3 position;
    float3 front;
    float3 up;
    float fov_y;
    float aspect_ratio;
    
    Camera(const float3& pos, const float3& front, const float3& up, float fov, float aspect)
      : position(pos), front(normalize(front)), up(normalize(up)), fov_y(fov), aspect_ratio(aspect) {}

    Ray get_ray(float x, float y) const {
        float3 right = normalize(cross(front, up));
        float3 actual_up = cross(right, front);
        float tan_fov = tan(fov_y * M_PI / 180.0f / 2.0f);
        float height_world = 2.0f * tan_fov;
        float width_world = aspect_ratio * height_world;
        float3 bottom_left = front - (width_world / 2.0f) * right - (height_world / 2.0f) * actual_up;
        float3 ray_dir = bottom_left + x * width_world * right + y * height_world * actual_up;
        return Ray{position, normalize(ray_dir)};
    }
};

Camera create_camera_from_data(const CameraData& camData) {
    float rad_yaw = camData.yaw * M_PI / 180.0f;
    float rad_pitch = camData.pitch * M_PI / 180.0f;
    float3 front = normalize(float3(
        cos(rad_pitch) * sin(rad_yaw),
        sin(rad_pitch),
        cos(rad_pitch) * cos(rad_yaw)
    ));
    return Camera(camData.position, front, camData.up, camData.fov_y, camData.aspect_ratio);
}

class Scene {
public:
    std::vector<std::unique_ptr<SceneObject>> objects;
    std::vector<Light> lights;

    bool intersect(const Ray& ray, Intersection& isect) const {
        bool hit = false;
        float t_min = std::numeric_limits<float>::max();
        for (const auto& obj : objects) {
            Intersection temp_isect;
            if (obj->intersect(ray, temp_isect) && temp_isect.t < t_min) {
                t_min = temp_isect.t;
                isect = temp_isect;
                hit = true;
            }
        }
        return hit;
    }

    bool intersect_any(const Ray& ray, float t_max) const {
        Intersection isect;
        return intersect(ray, isect) && isect.t < t_max;
    }
};

float3 trace(const Scene& scene, const Ray& ray, int depth, int max_depth) {
    if (depth > max_depth) return float3(0, 0, 0);
    Intersection isect;
    if (scene.intersect(ray, isect))
    {
        float3 color(0, 0, 0);
        if (isect.material.is_mirror) { // A5: Зеркальные отражения
            float3 reflect_dir = ray.direction - 2.0f * dot(ray.direction, isect.normal) * isect.normal;
            Ray reflect_ray(isect.position + 0.001f * isect.normal, normalize(reflect_dir));
            color = trace(scene, reflect_ray, depth + 1, max_depth);
        }
        else
        { // Диффузное освещение по Ламберту
            for (const auto& light : scene.lights) {
                float3 light_dir = normalize(light.position - isect.position);
                float3 shadow_origin = isect.position + 0.001f * isect.normal;
                Ray shadow_ray(shadow_origin, light_dir);
                float t_light = length(light.position - isect.position);
                if (!scene.intersect_any(shadow_ray, t_light)) { // A4: Тени
                    float ndotl = std::max(0.0f, dot(isect.normal, light_dir));
                    color += isect.material.diffuse_color * light.intensity * ndotl;
                }
            }
        }
        return clamp(color, float3(0.0f), float3(1.0f));
    }
    return float3(0, 0, 0);    // Цвет фона
}


void draw_frame(const AppData &app_data, const Camera &camera, std::vector<uint32_t> &pixels) {
    int width = app_data.width;
    int height = app_data.height;
    Scene scene;
    // A1: Плоскость (пол y = 0)
    auto plane = std::make_unique<Plane>(0, 1, 0, 0);
    plane->material.diffuse_color = float3(0.5, 0.5, 0.5);
    plane->material.is_mirror = false;
    scene.objects.push_back(std::move(plane));
    // Пример ещё одной плоскости со зеркальным эффектом
    plane = std::make_unique<Plane>(0, 0, 1, 2);
    plane->material.diffuse_color = float3(0.5, 0.5, 0.5);
    plane->material.is_mirror = false; ///////////////////////////////////////
    scene.objects.push_back(std::move(plane));
    // A2/A3: Меш (например, куб)
    auto mesh = std::make_unique<Mesh>();
    mesh->load_from_obj("cube.obj");
    mesh->material.diffuse_color = float3(0.0f, 1.0f, 0.0f);
    mesh->build_bvh();
    scene.objects.push_back(std::move(mesh));
    // Источники света
    Light light;
    light.position = float3(2, 0.5, 2);
    light.intensity = 1.0f;
    scene.lights.push_back(light);
    light.position = float3(2, 5, 2);
    light.intensity = 1.0f;
    scene.lights.push_back(light);


    for (int j = 0; j < height; j++) {    // Рендеринг сцены
        for (int i = 0; i < width; i++) {
            float x = (i + 0.5f) / width;
            float y = 1.0f - (j + 0.5f) / height;
            Ray ray = camera.get_ray(x, y);
            float3 color = trace(scene, ray, 0, 5);
            uint32_t r = static_cast<uint32_t>(color.x * 255);
            uint32_t g = static_cast<uint32_t>(color.y * 255);
            uint32_t b = static_cast<uint32_t>(color.z * 255);
            pixels[j * width + i] = (255 << 24) | (r << 16) | (g << 8) | b;
        }
    }
}




// -------------------------------------------------------------
// Часть B1
// -----------------------------------------------------------

float sdf_trilinear(const SdfGrid &grid, const float3 &p)
{
    //[-1,1] в [0, size-1]
    float3 gridPos = (p + float3(1,1,1)) * 0.5f;
    gridPos.x *= (grid.size.x - 1);
    gridPos.y *= (grid.size.y - 1);
    gridPos.z *= (grid.size.z - 1);

    int ix = static_cast<int>(floor(gridPos.x));
    int iy = static_cast<int>(floor(gridPos.y));
    int iz = static_cast<int>(floor(gridPos.z));
    float fx = gridPos.x - ix;
    float fy = gridPos.y - iy;
    float fz = gridPos.z - iz;

    auto index = [&](int x, int y, int z) -> int {
        return x + y * grid.size.x + z * grid.size.x * grid.size.y;
    };

    float v000 = grid.data[index(ix,   iy,   iz)];
    float v100 = grid.data[index(ix+1, iy,   iz)];
    float v010 = grid.data[index(ix,   iy+1, iz)];
    float v110 = grid.data[index(ix+1, iy+1, iz)];
    float v001 = grid.data[index(ix,   iy,   iz+1)];
    float v101 = grid.data[index(ix+1, iy,   iz+1)];
    float v011 = grid.data[index(ix,   iy+1, iz+1)];
    float v111 = grid.data[index(ix+1, iy+1, iz+1)];

    float v00 = v000 * (1 - fx) + v100 * fx;
    float v01 = v001 * (1 - fx) + v101 * fx;
    float v10 = v010 * (1 - fx) + v110 * fx;
    float v11 = v011 * (1 - fx) + v111 * fx;
    float v0 = v00 * (1 - fy) + v10 * fy;
    float v1 = v01 * (1 - fy) + v11 * fy;
    return v0 * (1 - fz) + v1 * fz;
}


float3 compute_sdf_normal(const SdfGrid &grid, const float3 &p, float eps = 0.001f)
{
    float dx = sdf_trilinear(grid, p + float3(eps,0,0)) - sdf_trilinear(grid, p - float3(eps,0,0));
    float dy = sdf_trilinear(grid, p + float3(0,eps,0)) - sdf_trilinear(grid, p - float3(0,eps,0));
    float dz = sdf_trilinear(grid, p + float3(0,0,eps)) - sdf_trilinear(grid, p - float3(0,0,eps));
    return normalize(float3(dx, dy, dz));
}

bool sphere_trace(const SdfGrid &grid, const Ray &ray, float &t_hit, int max_steps = 100, float eps = 0.001f)
{
    t_hit = 0.0f;
    for (int i = 0; i < max_steps; i++)
    {
        float3 p = ray.origin + t_hit * ray.direction;
        if (p.x < -1 || p.x > 1 || p.y < -1 || p.y > 1 || p.z < -1 || p.z > 1)
            return false;
        float d = sdf_trilinear(grid, p);
        if (fabs(d) < eps)
            return true;
        t_hit += d;
    }
    return false;
}

// Отрисовка SDF модели (B1)
void draw_frame_sdf(const AppData &app_data, const Camera &camera, const SdfGrid &grid, std::vector<uint32_t> &pixels)
{
    int width = app_data.width;
    int height = app_data.height;
    float3 lightPos = float3(2,2,2);
    float lightIntensity = 1.0f;
    for (int j = 0; j < height; j++)
    {
        for (int i = 0; i < width; i++)
        {
            float x = (i + 0.5f) / width;
            float y = 1.0f - (j + 0.5f) / height;
            Ray ray = camera.get_ray(x, y);
            float t_hit;
            float3 color(0,0,0);
            if (sphere_trace(grid, ray, t_hit))
            {
                float3 p = ray.origin + t_hit * ray.direction;
                float3 normal = compute_sdf_normal(grid, p);
                float3 light_dir = normalize(lightPos - p);
                float ndotl = std::max(0.0f, dot(normal, light_dir));
                color = float3(1.0f, 1.0f, 1.0f) * lightIntensity * ndotl;
            }
            uint32_t r = static_cast<uint32_t>(LiteMath::clamp(color.x, 0.0f, 1.0f)*255);
            uint32_t g = static_cast<uint32_t>(LiteMath::clamp(color.y, 0.0f, 1.0f)*255);
            uint32_t b = static_cast<uint32_t>(LiteMath::clamp(color.z, 0.0f, 1.0f)*255);
            pixels[j * width + i] = (255 << 24) | (r << 16) | (g << 8) | b;
        }
    }
}

// -----------------------------------------------
// Часть B2
// ----------------------------------------------


float3 closest_point_on_triangle(const float3 &p, const float3 &a, const float3 &b, const float3 &c)
{
    float3 ab = b - a;
    float3 ac = c - a;
    float3 ap = p - a;
    float d1 = dot(ab, ap);
    float d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    float3 bp = p - b;
    float d3 = dot(ab, bp);
    float d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    float vc = d1*d4 - d3*d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        float v = d1 / (d1 - d3);
        return a + v * ab;
    }
    float3 cp = p - c;
    float d5 = dot(ab, cp);
    float d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    float vb = d5*d2 - d1*d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        float w = d2 / (d2 - d6);
        return a + w * ac;
    }
    float va = d3*d6 - d5*d4;
    if (va <= 0)
    {
        float w = d4 / (d4 - d5);
        return b + w * (c - b);
    }
    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    return a + ab * v + ac * w;
}


float mesh_sdf_at_point(const Mesh &mesh, const float3 &p)
{
    float bestDist = std::numeric_limits<float>::max();
    float signResult = 1.0f;
    for (size_t i = 0; i < mesh.indices.size(); i++)
    {
        const int3 &idx = mesh.indices[i];
        float3 a = mesh.vertices[idx.x];
        float3 b = mesh.vertices[idx.y];
        float3 c = mesh.vertices[idx.z];
        float3 pt = closest_point_on_triangle(p, a, b, c);
        float dist = length(p - pt);
        if (dist < bestDist)
        {
            bestDist = dist;
            float3 triNormal = normalize(cross(b - a, c - a));
            signResult = (dot(triNormal, p - pt) < 0) ? -1.0f : 1.0f;
        }
    }
    return signResult * bestDist;
}


void build_sdf_grid_from_mesh(const Mesh &mesh, SdfGrid &grid, int resolution)
{
    grid.size = uint3(resolution, resolution, resolution);
    grid.data.resize(resolution * resolution * resolution);
    for (int z = 0; z < resolution; z++)
    {
        for (int y = 0; y < resolution; y++)
        {
            for (int x = 0; x < resolution; x++)
            {
                float fx = static_cast<float>(x) / (resolution - 1);
                float fy = static_cast<float>(y) / (resolution - 1);
                float fz = static_cast<float>(z) / (resolution - 1);
                float3 p = float3(fx, fy, fz) * 2.0f - float3(1,1,1);
                grid.data[x + y * resolution + z * resolution * resolution] = mesh_sdf_at_point(mesh, p);
            }
        }
    }
}

// --------------------------------------------------------------
// Часть B3
// -----------------------------------------------------------------

void modify_sdf_grid(SdfGrid &grid, const float3 &sphere_center, float sphere_radius)
{
    int resX = grid.size.x, resY = grid.size.y, resZ = grid.size.z;
    for (int z = 0; z < resZ; z++)
    {
        for (int y = 0; y < resY; y++)
        {
            for (int x = 0; x < resX; x++)
            {
                float fx = static_cast<float>(x) / (resX - 1);
                float fy = static_cast<float>(y) / (resY - 1);
                float fz = static_cast<float>(z) / (resZ - 1);
                float3 p = float3(fx, fy, fz) * 2.0f - float3(1,1,1);
                float sphereSDF = length(p - sphere_center) - sphere_radius;
                int idx = x + y * resX + z * resX * resY;
                grid.data[idx] = std::max(grid.data[idx], -sphereSDF);
            }
        }
    }
}


float sphere_sdf(const float3 &p, const float3 &center, float radius) {
    return length(p - center) - radius;
}

bool node_has_surface(const SdfOctreeNode &node) {
    float first_sign = node.values[0] < 0 ? -1.0f : 1.0f;
    for (int i = 1; i < 8; i++) {
        if ((node.values[i] < 0 ? -1.0f : 1.0f) != first_sign) {
            return true;
        }
    }
    return false;
}


float sdf_octree_at_point(const SdfOctree &octree, const float3 &p, int max_depth) {
    if (p.x < -1 || p.x > 1 || p.y < -1 || p.y > 1 || p.z < -1 || p.z > 1) {
        return 1.0f;
    }
    if (octree.nodes.empty()) return 1.0f;

    float3 node_min = float3(-1, -1, -1);
    float node_size = 2.0f;
    size_t node_idx = 0;
    int depth = 0;

    while (depth < max_depth && octree.nodes[node_idx].offset != 0) {
        if (!node_has_surface(octree.nodes[node_idx])) {
            return 1.0f;
        }

        float half_size = node_size * 0.5f;
        float3 node_center = node_min + float3(half_size, half_size, half_size);

        int child_x = p.x >= node_center.x ? 1 : 0;
        int child_y = p.y >= node_center.y ? 1 : 0;
        int child_z = p.z >= node_center.z ? 1 : 0;
        int child_idx = child_x + (child_y << 1) + (child_z << 2);

        node_min.x += child_x * half_size;
        node_min.y += child_y * half_size;
        node_min.z += child_z * half_size;
        node_size = half_size; // Обновляем размер узла

        node_idx = octree.nodes[node_idx].offset + child_idx;
        depth++;

        if (node_idx >= octree.nodes.size()) {
            return 1.0f;
        }
    }

    const SdfOctreeNode &node = octree.nodes[node_idx];
    float3 local_p = (p - node_min) / node_size;
    float fx = local_p.x;
    float fy = local_p.y;
    float fz = local_p.z;

    float v00 = node.values[0] * (1 - fx) + node.values[1] * fx;
    float v01 = node.values[2] * (1 - fx) + node.values[3] * fx;
    float v10 = node.values[4] * (1 - fx) + node.values[5] * fx;
    float v11 = node.values[6] * (1 - fx) + node.values[7] * fx;
    float v0 = v00 * (1 - fy) + v01 * fy;
    float v1 = v10 * (1 - fy) + v11 * fy;
    return v0 * (1 - fz) + v1 * fz;
}

float3 compute_sdf_octree_normal(const SdfOctree &octree, const float3 &p, int max_depth, float eps = 0.0001f) {
    float d0 = sdf_octree_at_point(octree, p, max_depth);
    float dx = sdf_octree_at_point(octree, p + float3(eps, 0, 0), max_depth) - d0;
    float dy = sdf_octree_at_point(octree, p + float3(0, eps, 0), max_depth) - d0;
    float dz = sdf_octree_at_point(octree, p + float3(0, 0, eps), max_depth) - d0;
    return normalize(float3(dx, dy, dz));
}


bool sphere_trace_octree(const SdfOctree &octree, const Ray &ray, float &t_hit, int max_depth,
                         int max_steps = 100, float eps = 0.001f) {
    t_hit = 0.0f;
    const float MAX_DIST = 10.0f;
    for (int i = 0; i < max_steps; i++) {
        float3 p = ray.origin + t_hit * ray.direction;
        if (p.x < -1 || p.x > 1 || p.y < -1 || p.y > 1 || p.z < -1 || p.z > 1) {
            return false;
        }
        float d = sdf_octree_at_point(octree, p, max_depth);
        if (d < eps) {
            return true;
        }
        t_hit += d;
        if (t_hit < 0 || t_hit > MAX_DIST) {
            return false;
        }
    }
    return false;
}

void draw_frame_octree(const AppData &app_data, const Camera &camera, const SdfOctree &octree,
                       int max_depth, std::vector<uint32_t> &pixels) {
    int width = app_data.width;
    int height = app_data.height;
    float3 lightPos = float3(2, 2, 2);
    float lightIntensity = 1.0f;

    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            float x = (i + 0.5f) / width;
            float y = 1.0f - (j + 0.5f) / height;
            Ray ray = camera.get_ray(x, y);
            float t_hit;
            float3 color(0, 0, 0);
            if (sphere_trace_octree(octree, ray, t_hit, max_depth)) {
                float3 p = ray.origin + t_hit * ray.direction;
                float3 normal = compute_sdf_octree_normal(octree, p, max_depth);
                float3 light_dir = normalize(lightPos - p);
                float ndotl = std::max(0.0f, dot(normal, light_dir));
                color = float3(1.0f, 1.0f, 1.0f) * lightIntensity * ndotl;
            }
            uint32_t r = static_cast<uint32_t>(LiteMath::clamp(color.x, 0.0f, 1.0f) * 255);
            uint32_t g = static_cast<uint32_t>(LiteMath::clamp(color.y, 0.0f, 1.0f) * 255);
            uint32_t b = static_cast<uint32_t>(LiteMath::clamp(color.z, 0.0f, 1.0f) * 255);
            pixels[j * width + i] = (255 << 24) | (r << 16) | (g << 8) | b;
        }
    }
}


struct OctreeBuildNode {
    float3 min;    // Минимальная граница узла
    float size;   // Размер узла
    size_t idx;  // Индекс в векторе nodes
};

void build_sdf_octree_from_mesh(const Mesh &mesh, SdfOctree &octree, int max_depth) {
    octree.nodes.clear();
    if (max_depth < 1) return;

    octree.nodes.push_back(SdfOctreeNode());
    std::vector<OctreeBuildNode> stack;
    stack.push_back({float3(-1, -1, -1), 2.0f, 0});

    while (!stack.empty()) {
        OctreeBuildNode current = stack.back();
        stack.pop_back();
        SdfOctreeNode &node = octree.nodes[current.idx];

        // SDF в углах узла в правильном порядке:
        // (0,0,0), (1,0,0), (0,1,0), (1,1,0), (0,0,1), (1,0,1), (0,1,1), (1,1,1)
        for (int i = 0; i < 8; i++) {
            float3 corner = current.min;
            if (i & 1) corner.x += current.size; // x
            if (i & 2) corner.y += current.size; // y
            if (i & 4) corner.z += current.size; // z
            node.values[i] = mesh_sdf_at_point(mesh, corner);
        }

        bool has_surface = node_has_surface(node);
        int current_depth = static_cast<int>(log2(2.0f / current.size));
        if (!has_surface || current_depth >= max_depth) {
            node.offset = 0;
            continue;
        }

        node.offset = octree.nodes.size();
        float half_size = current.size * 0.5f;
        for (int i = 0; i < 8; i++) {
            octree.nodes.push_back(SdfOctreeNode());
            float3 child_min = current.min;
            if (i & 1) child_min.x += half_size;
            if (i & 2) child_min.y += half_size;
            if (i & 4) child_min.z += half_size;
            stack.push_back({child_min, half_size, node.offset + i});
        }
    }
}



void modify_sdf_octree(SdfOctree &octree, const float3 &sphere_center, float sphere_radius, int max_depth) {
    if (octree.nodes.empty()) return;

    std::vector<bool> keep_nodes(octree.nodes.size(), true);
    std::vector<OctreeBuildNode> stack;
    stack.push_back({float3(-1, -1, -1), 2.0f, 0});

    while (!stack.empty()) {
        OctreeBuildNode current = stack.back();
        stack.pop_back();
        SdfOctreeNode &node = octree.nodes[current.idx];

        for (int i = 0; i < 8; i++) {
            float3 corner = current.min;
            if (i & 1) corner.x += current.size;
            if (i & 2) corner.y += current.size;
            if (i & 4) corner.z += current.size;
            float sphere_sdf_val = sphere_sdf(corner, sphere_center, sphere_radius);
            node.values[i] = node.values[i] - sphere_sdf_val;
        }

        if (node.offset == 0) continue;

        int current_depth = static_cast<int>(log2(2.0f / current.size));
        if (!node_has_surface(node) || current_depth >= max_depth) {
            for (size_t i = node.offset; i < node.offset + 8; i++) {
                if (i < octree.nodes.size()) {
                    keep_nodes[i] = false;
                }
            }
            node.offset = 0;
            continue;
        }

        float half_size = current.size * 0.5f;
        for (int i = 0; i < 8; i++) {
            float3 child_min = current.min;
            if (i & 1) child_min.x += half_size;
            if (i & 2) child_min.y += half_size;
            if (i & 4) child_min.z += half_size;
            stack.push_back({child_min, half_size, node.offset + i});
        }
    }

    std::vector<SdfOctreeNode> new_nodes;
    std::vector<size_t> old_to_new(octree.nodes.size(), 0);
    for (size_t i = 0; i < octree.nodes.size(); i++) {
        if (keep_nodes[i]) {
            old_to_new[i] = new_nodes.size();
            new_nodes.push_back(octree.nodes[i]);
        }
    }

    for (auto &node : new_nodes) {
        if (node.offset != 0) {
            node.offset = old_to_new[node.offset];
        }
    }

    octree.nodes = std::move(new_nodes);
}


// -----------------------------------------------------------------
// Основная функция: выбор режима рендеринга, управление камерой и параметрами сферы (B3)
// -----------------------------------------------------------------
int main(int argc, char **args)
{
  const int SCREEN_WIDTH = 512;
  const int SCREEN_HEIGHT = 512;
  std::vector<uint32_t> pixels(SCREEN_WIDTH * SCREEN_HEIGHT, 0xFFFFFFFF);
  AppData app_data;
  app_data.width = SCREEN_WIDTH;
  app_data.height = SCREEN_HEIGHT;
  load_sdf_grid(app_data.loaded_grid, "example_grid.grid");
  SdfOctree octree;
  int octree_depth = 8;
  if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
  {
    std::cerr << "Ошибка инициализации SDL: " << SDL_GetError() << std::endl;
    return 1;
  }

  SDL_Window *window = SDL_CreateWindow("SDF Viewer", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                        SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);

  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

  SDL_Texture *texture = SDL_CreateTexture(
      renderer,
      SDL_PIXELFORMAT_ARGB8888,
      SDL_TEXTUREACCESS_STREAMING,
      SCREEN_WIDTH,
      SCREEN_HEIGHT);


  // Инициализация данных камеры
  CameraData camData;
  camData.position = float3(0.5, 0.5, 0.5);
  camData.yaw = 180.0f;
  camData.pitch = 0.0f;
  camData.up = float3(0, 1, 0);
  camData.fov_y = 80.0f;
  camData.aspect_ratio = static_cast<float>(SCREEN_WIDTH) / SCREEN_HEIGHT;

  float3 sphereCenter = float3(0, 0.5f, 0);
  float sphereRadius = 0.7f;

  SDL_Event ev;
  bool running = true;
  const float moveSpeed = 0.1f;
  const float turnSpeed = 2.0f;
  // Переключение между режимами рендеринга:
  // '1' - стандартный рендер сцены (меш)
  // '2' - рендер SDF модели с sphere tracing (B1)
  // '3' - построение SDF сетки по мешу (B2) с последующей модификацией (B3)
  // '4' - None не работает(
  int renderMode = 1;

  while (running)
  {
    while (SDL_PollEvent(&ev) != 0)
    {
      if (ev.type == SDL_QUIT)
      {
        running = false;
      }
      else if (ev.type == SDL_KEYDOWN)
      {
        switch (ev.key.keysym.sym)
        {
          // Перемещение камеры (WASD)
          case SDLK_w:
            {
              float rad_yaw = camData.yaw * M_PI / 180.0f;
              camData.position.x += moveSpeed * sin(rad_yaw);
              camData.position.z += moveSpeed * cos(rad_yaw);
            }
            break;
          case SDLK_s:
            {
              float rad_yaw = camData.yaw * M_PI / 180.0f;
              camData.position.x -= moveSpeed * sin(rad_yaw);
              camData.position.z -= moveSpeed * cos(rad_yaw);
            }
            break;
          case SDLK_a:
            {
              float rad_yaw = camData.yaw * M_PI / 180.0f;
              camData.position.x += moveSpeed * cos(rad_yaw);
              camData.position.z -= moveSpeed * sin(rad_yaw);
            }
            break;
          case SDLK_d:
            {
              float rad_yaw = camData.yaw * M_PI / 180.0f;
              camData.position.x -= moveSpeed * cos(rad_yaw);
              camData.position.z += moveSpeed * sin(rad_yaw);
            }
            break;
          // Поворот камеры (стрелки)
          case SDLK_UP:
            camData.pitch += turnSpeed;
            if (camData.pitch > 89.0f) camData.pitch = 89.0f;
            break;
          case SDLK_DOWN:
            camData.pitch -= turnSpeed;
            if (camData.pitch < -89.0f) camData.pitch = -89.0f;
            break;
          case SDLK_LEFT:
            camData.yaw += turnSpeed;
            break;
          case SDLK_RIGHT:
            camData.yaw -= turnSpeed;
            break;
          case SDLK_1:
            renderMode = 1;
            break;
          case SDLK_2:
            renderMode = 2;
            break;
          case SDLK_3:
            renderMode = 3;
            break;
          case SDLK_4:
            renderMode = 4;
            break;
          case SDLK_EQUALS:
            sphereRadius += 0.05f;
            break;
          case SDLK_MINUS:
            sphereRadius = std::max(0.05f, sphereRadius - 0.05f);
            break;
          case SDLK_ESCAPE:
            running = false;
            break;
          default:
            break;
        }
      }
    }

    Camera camera = create_camera_from_data(camData);

    if (renderMode == 1)
      draw_frame(app_data, camera, pixels);
    else if (renderMode == 2)
      draw_frame_sdf(app_data, camera, app_data.loaded_grid, pixels);
    else if (renderMode == 3)
    {
      // Строим SDF сетку по мешу (B2)
      Mesh mesh;
      mesh.load_from_obj("cube.obj");
      mesh.build_bvh();
      SdfGrid generatedGrid;
      int resolution = 128;
      build_sdf_grid_from_mesh(mesh, generatedGrid, resolution);
      // Применяем модификацию (B3): вырезаем сферу
      modify_sdf_grid(generatedGrid, sphereCenter, sphereRadius);
      draw_frame_sdf(app_data, camera, generatedGrid, pixels);
    }
    else if (renderMode == 4) {
    // Режим C: SDF Octree
    Mesh mesh;
    mesh.load_from_obj("cube.obj");
    build_sdf_octree_from_mesh(mesh, octree, octree_depth);
    //modify_sdf_octree(octree, sphereCenter, sphereRadius, octree_depth);
    draw_frame_octree(app_data, camera, octree, octree_depth, pixels);
    }

    SDL_UpdateTexture(texture, nullptr, pixels.data(), SCREEN_WIDTH * sizeof(uint32_t));
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, nullptr, nullptr);
    SDL_RenderPresent(renderer);
  }

  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}

