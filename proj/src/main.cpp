////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <stack>

//Perlin Noise
#include "PerlinNoise.h"
#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>

// Eigen for matrix operations
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"
#include "utils.h"

// JSON parser library (https://github.com/nlohmann/json)
#include "json.hpp"
using json = nlohmann::json;

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

int whichObject = 1; //global variable to keep track of which variable we're raytracing

int xPixel = 0;
int yPixel = 0;


////////////////////////////////////////////////////////////////////////////////
// Define types & classes
////////////////////////////////////////////////////////////////////////////////

struct Ray {
    Vector3d origin;
    Vector3d direction;
    Ray() { }
    Ray(Vector3d o, Vector3d d) : origin(o), direction(d) { }
};

struct Light {
    Vector3d position;
    Vector3d intensity;
};

struct Intersection {
    Vector3d position;
    Vector3d normal;
    double ray_param;
};

struct Camera {
    bool is_perspective;
    Vector3d position;
    double field_of_view; // between 0 and PI
    double focal_length;
    double lens_radius; // for depth of field
};

struct Material {
    Vector3d ambient_color;
    Vector3d diffuse_color;
    Vector3d specular_color;
    double specular_exponent; 

    Vector3d reflection_color;
    Vector3d refraction_color;
    double refraction_index;
};

struct Object {
    Material material;
    virtual ~Object() = default;
    virtual bool intersect(const Ray &ray, Intersection &hit) = 0;
};

// Use smart pointers to hold objects as this is a virtual class
typedef std::shared_ptr<Object> ObjectPtr;

struct Sphere : public Object {
    Vector3d position;
    double radius;

    virtual ~Sphere() = default;
    virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct Parallelogram : public Object {
    Vector3d origin;
    Vector3d u;
    Vector3d v;

    virtual ~Parallelogram() = default;
    virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct AABBTree {
    struct Node {
        AlignedBox3d bbox;
        int parent; // Index of the parent node (-1 for root)
        int left; // Index of the left child (-1 for a leaf)
        int right; // Index of the right child (-1 for a leaf)
        int triangle; // Index of the node triangle (-1 for internal nodes)
    };

    std::vector<Node> nodes;
    int root;

    AABBTree() = default; // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh
};

struct Mesh : public Object {
    MatrixXd vertices; // n x 3 matrix (n points)
    MatrixXi facets; // m x 3 matrix (m triangles)

    AABBTree bvh;

    Mesh() = default; // Default empty constructor
    Mesh(const std::string &filename);
    virtual ~Mesh() = default;
    virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct Scene {
    Vector3d ground;
    Vector3d background_color;
    Vector3d ambient_light;

    Camera camera;
    std::vector<Material> materials;
    std::vector<Light> lights;
    std::vector<ObjectPtr> objects;
};

////////////////////////////////////////////////////////////////////////////////

// Read a triangle mesh from an off file
void load_off(const std::string &filename, MatrixXd &V, MatrixXi &F) {
    std::ifstream in(filename);
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    V.resize(nv, 3);
    F.resize(nf, 3);
    for (int i = 0; i < nv; ++i) {
        in >> V(i, 0) >> V(i, 1) >> V(i, 2);
    }
    for (int i = 0; i < nf; ++i) {
        int s;
        in >> s >> F(i, 0) >> F(i, 1) >> F(i, 2);
        assert(s == 3);
    }
}


void set_scale_translate(int &whichObject, Matrix4f &scale, Matrix4f &translate) {
    if (whichObject==1) { //table
        scale << 0.22,0,0,0,
                 0,0.22,0,0,
                 0,0,0.22,0,
                 0,0,0,1;

        translate << 1,0,0,0.4,
                     0,1,0,0.4,
                     0,0,1,0.4,
                     0,0,0,1;
    }
    else if (whichObject==2) { //elephant
        scale << 0.01,0,0,0,
                 0,0.01,0,0,
                 0,0,0.01,0,
                 0,0,0,1;

        translate << 1,0,0,0.797,
                     0,1,0,0.797,
                     0,0,1,0.797,
                     0,0,0,1;
    }
    else if (whichObject==3) { //bunny
        scale << 7.22,0,0,0,
                 0,7.22,0,0,
                 0,0,7.22,0,
                 0,0,0,1;

        translate << 1,0,0,0.32,
                     0,1,0,0.32,
                     0,0,1,0.32,
                     0,0,0,1;
    }
    else if (whichObject==4) { //sphere
        scale << 98.3,0,0,0,
                 0,98.3,0,0,
                 0,0,98.3,0,
                 0,0,0,1;

        translate << 1,0,0,-0.35,
                     0,1,0,-0.35,
                     0,0,1,-0.35,
                     0,0,0,1;   
    }
    else if (whichObject==5) { //monk
        scale << 0.167,0,0,0,
                 0,0.167,0,0,
                 0,0,0.167,0,
                 0,0,0,1;

        translate << 1,0,0,-.287,
                     0,1,0,-.287,
                     0,0,1,-.287,
                     0,0,0,1;
    }
    
    whichObject++;
}


Mesh::Mesh(const std::string &filename) {
    load_off(filename, vertices, facets);

    Matrix4f scale;
    Matrix4f translate;

    set_scale_translate(whichObject, scale, translate);

    for (double i=0; i<vertices.size(); i+=3) {
        Vector4f originalVertices(vertices(0+i),vertices(1+i),vertices(2+i),1);
        Vector4f newVertices = scale*originalVertices;  //matrix multiplcation to scale
        newVertices = translate*newVertices;    //matrix multiplcation to translate

        //update original vertices
        vertices(0+i) = newVertices(0);
        vertices(1+i) = newVertices(1);
        vertices(2+i) = newVertices(2);
    }

    bvh = AABBTree(vertices, facets);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Implementation
////////////////////////////////////////////////////////////////////////////////

AlignedBox3d bbox_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c) {
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F) {
    // Compute the centroids of all the triangles in the input mesh
    MatrixXd centroids(F.rows(), V.cols());
    centroids.setZero();
    for (int i = 0; i < F.rows(); ++i) {
        for (int k = 0; k < F.cols(); ++k) {
            centroids.row(i) += V.row(F(i, k));
        }
        centroids.row(i) /= F.cols();
    }

    // Top-down approach: split each set of primitives into 2 sets of roughly equal size,
    // based on sorting the centroids along one direction or another.
    std::vector<int> triangles(F.rows());
    std::iota(triangles.begin(), triangles.end(), 0);

    std::function<int(int, int, int)> top_down = [&] (int i, int j, int parent) {
        // Scene is empty, so is the AABB tree
        if (j - i == 0) {
            return -1;
        }

        // If there is only 1 triangle left, then we are at a leaf
        if (j - i == 1) {
            Node node;
            int f = triangles[i];
            RowVector3d a = V.row(F(f, 0));
            RowVector3d b = V.row(F(f, 1));
            RowVector3d c = V.row(F(f, 2));
            node.bbox = bbox_triangle(a, b, c);
            node.parent = parent;
            node.left = node.right = -1;
            node.triangle = triangles[i];
            nodes.push_back(node);
            return (int) (nodes.size() - 1);
        }

        // Otherwise, need to sort centroids along the longest dimension, and split recursively
        AlignedBox3d centroid_box;
        for (int k = i; k < j; ++k) {
            Vector3d c = centroids.row(triangles[k]).transpose();
            centroid_box.extend(c);
        }
        Vector3d extent = centroid_box.diagonal();
        int longest_dim = 0;
        for (int dim = 1; dim < 3; ++dim) {
            if (extent(dim) > extent(longest_dim)) {
                longest_dim = dim;
            }
        }
        std::sort(triangles.begin() + i, triangles.begin() + j, [&] (int f1, int f2) {
            return centroids(f1, longest_dim) < centroids(f2, longest_dim);
        });

        // Then create a new internal node
        int current = nodes.size();
        nodes.resize(current + 1);
        int midpoint = (i + j) / 2;
        int left = top_down(i, midpoint, current);
        int right = top_down(midpoint, j, current);
        Node &node = nodes[current];
        node.left = left;
        node.right = right;
        node.parent = parent;
        node.triangle = -1;
        node.bbox = nodes[node.left].bbox.extend(nodes[node.right].bbox);

        return current;
    };

    root = top_down(0, triangles.size(), -1);

    // Bottom-up approach: merge nodes 2 by 2, starting from the leaves of the tree,
    // until only 1 tree is left.
}

////////////////////////////////////////////////////////////////////////////////

bool Sphere::intersect(const Ray &ray, Intersection &hit) {
    Vector3d o = ray.origin - this->position;
    Vector3d d = ray.direction;
    double A = d.dot(d);
    double B = 2.0 * d.dot(o);
    double C = o.dot(o) - this->radius * this->radius;

    double delta = B*B - 4.0*A*C;
    if (delta < 0) {
        return false;
    } else {
        double t1 = (-B + std::sqrt(delta)) / (2.0 * A);
        double t2 = (-B - std::sqrt(delta)) / (2.0 * A);

        hit.ray_param = std::min(t1, t2);
        if (hit.ray_param < 0) {
            hit.ray_param = std::max(t1, t2);
        }
        if (hit.ray_param < 0) {
            return false;
        }

        hit.position = ray.origin + hit.ray_param * ray.direction;
        hit.normal = (hit.position - this->position).normalized();

        return true;
    }

    return false;
}

bool Parallelogram::intersect(const Ray &ray, Intersection &hit) {
    return false;
}

// -----------------------------------------------------------------------------

bool intersect_triangle(const Ray &ray, const Vector3d &a, const Vector3d &b, const Vector3d &c, Intersection &hit) {
    Matrix3d M;
    Vector3d y;
    M.col(0) = ray.direction;
    M.col(1) = a - b;
    M.col(2) = a - c;
    y = a - ray.origin;
    Vector3d x = M.colPivHouseholderQr().solve(y);
    double t = x(0);
    double u = x(1);
    double v = x(2);
    if (u >= 0 && v >= 0 && u + v <= 1 && t > 0) {
        if (!M.fullPivLu().isInvertible()) {
            return false;
        }
        hit.ray_param = t;
        hit.position = ray.origin + hit.ray_param * ray.direction;
        hit.normal = (b - a).cross(c - a).normalized();
        return true;
    } else {
        return false;
    }
}

bool intersect_box(const Ray &ray, const AlignedBox3d &box) {
    double tmin = 0;
    double tmax = std::numeric_limits<double>::max();
    for (int dim = 0; dim < 3; ++dim) {
        if (std::abs(ray.direction(dim)) < 1e-5) {
            if (ray.origin[dim] < box.min()[dim] || ray.origin[dim] > box.max()[dim]) {
                return false;
            }
        } else {
            double t1 = (box.min()[dim] - ray.origin[dim]) / ray.direction[dim];
            double t2 = (box.max()[dim] - ray.origin[dim]) / ray.direction[dim];
            tmin = std::max(std::min(t1, t2), tmin);
            tmax = std::min(std::max(t1, t2), tmax);
        }
    }
    return (tmin <= tmax);
}

bool Mesh::intersect(const Ray &ray, Intersection &closest_hit) {
    // Leverages the BVH
    bool hit_exists = false;
    closest_hit.ray_param = std::numeric_limits<double>::max();
    std::stack<int> q;
    q.push(bvh.root);
    while (!q.empty()) {
        const AABBTree::Node &node = bvh.nodes[q.top()];
        q.pop();
        if (intersect_box(ray, node.bbox)) {
            if (node.left < 0) {
                // Node is a leaf, we need to test the intersection with its triangle
                RowVector3d a = vertices.row(facets(node.triangle, 0));
                RowVector3d b = vertices.row(facets(node.triangle, 1));
                RowVector3d c = vertices.row(facets(node.triangle, 2));
                Intersection tmp_hit;
                if (intersect_triangle(ray, a, b, c, tmp_hit)) {
                    if (tmp_hit.ray_param < closest_hit.ray_param) {
                        closest_hit = tmp_hit;
                        hit_exists = true;
                    }
                }
            } else {
                // Internal node, we need to test intersection with both children
                q.push(node.left);
                q.push(node.right);
            }
        }
    }
    return hit_exists;
}

////////////////////////////////////////////////////////////////////////////////
// Define ray-tracing functions
////////////////////////////////////////////////////////////////////////////////

// Function declaration here (could be put in a header file)
Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &object, const Intersection &hit, int max_bounce);
Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &hit, double &closest_t);
bool is_light_visible(const Scene &scene, const Ray &ray, const Light &light);
Vector3d shoot_ray(const Scene &scene, const Ray &ray, int max_bounce);

// -----------------------------------------------------------------------------


// Set up Perlin Noise


// Initialize with the reference values for the permutation vector
PerlinNoise::PerlinNoise() {
    // Initialize the permutation vector with the reference values
    p = {
        151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,
        8,99,37,240,21,10,23,190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,
        35,11,32,57,177,33,88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,
        134,139,48,27,166,77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,
        55,46,245,40,244,102,143,54, 65,25,63,161,1,216,80,73,209,76,132,187,208, 89,
        18,169,200,196,135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,
        250,124,123,5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,
        189,28,42,223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 
        43,172,9,129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,
        97,228,251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,
        107,49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
        138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180 };
    // Duplicate the permutation vector
    p.insert(p.end(), p.begin(), p.end());
}

// Generate a new permutation vector based on the value of seed
PerlinNoise::PerlinNoise(unsigned int seed) {
    p.resize(256);

    // Fill p with values from 0 to 255
    std::iota(p.begin(), p.end(), 0);

    // Initialize a random engine with seed
    std::default_random_engine engine(seed);

    // Suffle  using the above random engine
    std::shuffle(p.begin(), p.end(), engine);

    // Duplicate the permutation vector
    p.insert(p.end(), p.begin(), p.end());
}

double PerlinNoise::noise(double x, double y, double z) {
    // Find the unit cube that contains the point
    int X = (int) floor(x) & 255;
    int Y = (int) floor(y) & 255;
    int Z = (int) floor(z) & 255;

    // Find relative x, y,z of point in cube
    x -= floor(x);
    y -= floor(y);
    z -= floor(z);

    // Compute fade curves for each of x, y, z
    double u = fade(x);
    double v = fade(y);
    double w = fade(z);

    // Hash coordinates of the 8 cube corners
    int A = p[X] + Y;
    int AA = p[A] + Z;
    int AB = p[A + 1] + Z;
    int B = p[X + 1] + Y;
    int BA = p[B] + Z;
    int BB = p[B + 1] + Z;

    // Add blended results from 8 corners of cube
    double res = lerp(w, lerp(v, lerp(u, grad(p[AA], x, y, z), grad(p[BA], x-1, y, z)), lerp(u, grad(p[AB], x, y-1, z), grad(p[BB], x-1, y-1, z))), lerp(v, lerp(u, grad(p[AA+1], x, y, z-1), grad(p[BA+1], x-1, y, z-1)), lerp(u, grad(p[AB+1], x, y-1, z-1),  grad(p[BB+1], x-1, y-1, z-1))));
    return (res + 1.0)/2.0;
}

double PerlinNoise::fade(double t) { 
    return t * t * t * (t * (t * 6 - 15) + 10);
}

double PerlinNoise::lerp(double t, double a, double b) { 
    return a + t * (b - a); 
}

double PerlinNoise::grad(int hash, double x, double y, double z) {
    int h = hash & 15;
    // Convert lower 4 bits of hash into 12 gradient directions
    double u = h < 8 ? x : y,
           v = h < 4 ? y : h == 12 || h == 14 ? x : z;
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}


void set_object_color(const Ray &ray, const Intersection &hit, Vector3d &lights_color, Vector3d &refraction_color, const Light &light, const Material &mat) {   

    Vector3d Li = (light.position - hit.position).normalized();
    Vector3d N = hit.normal;

    // Diffuse contribution
    Vector3d diffuse = mat.diffuse_color * std::max(Li.dot(N), 0.0);
    // Specular contribution;
    Vector3d Hi = (Li - ray.direction).normalized();
    Vector3d specular = mat.specular_color * std::pow(std::max(N.dot(Hi), 0.0), mat.specular_exponent);


    if (whichObject==0) { //table
        // Create a PerlinNoise object with a random permutation vector generated with seed
        unsigned int seed = 237;
        PerlinNoise pn(seed);
        unsigned int kk = 0;
        double width = 64;
        double height = 48;

        double x = (double)xPixel/((double)width);
        double y = (double)yPixel/((double)height);

        double n = pn.noise(10 * x, 10 * y, 0.8);

        Vector3d d(n,n,n);
        diffuse = d * std::max(Li.dot(N), 0.0);
        specular = mat.specular_color * std::pow(std::max(N.dot(Hi), 0.0), 10000);
    }
    else if (whichObject==1) { //elephant
        Vector3d d(0.9,0.6,0.4);
        diffuse = d * std::max(Li.dot(N), 0.0);
    }
    else if (whichObject==2) { //bunny
        Vector3d d(0.9,0.9,0.4);
        diffuse = d * std::max(Li.dot(N), 0.0);
    }
    else if (whichObject==3) { //surface
        // Use Perlin Noise to simulate grass
        unsigned int seed = 237;
        PerlinNoise pn(seed);
        unsigned int kk = 0;
        double width = 164;
        double height = 98;

        double x = (double)xPixel/((double)width);
        double y = (double)yPixel/((double)height);

        double n = pn.noise(x, 10*y, 0.8);

        Vector3d d(n+0.05,n+0.35,n+0.001);
        diffuse = d * std::max(Li.dot(N), 0.0);
    }
    else if (whichObject==4) { //orb
        Vector3d d(0.566, 0.560, 0.515);
        diffuse = Vector3d(0.566, 0.560, 0.515) * std::max(Li.dot(N), 0.0); 
    
        Vector3d s(1, 1, 1);
        specular = s * std::pow(std::max(N.dot(Hi), 0.0), 14.5); 

        Vector3d refract(0.01,0.01,0.02);
        refraction_color = 1.43*refract*hit.normal.dot(hit.position-light.position); 
    }
    else if (whichObject==5) { //monk
        diffuse = Vector3d(2.1,2.1,2.1) * std::max(Li.dot(N), 0.0);
    }


    // Attenuate lights according to the squared distance to the lights
    Vector3d D = light.position - hit.position;
    lights_color += (diffuse + specular).cwiseProduct(light.intensity) /  D.squaredNorm();
}


void set_orb_reflection(const Scene &scene, const Ray &ray, const Intersection &hit, Vector3d &reflection_color, int max_bounce, const Material &mat) {
    Vector3d r = ray.direction - 2.0 * ray.direction.dot(hit.normal) * hit.normal;
    Ray reflect(hit.position+0.001*r, r);
    reflection_color = mat.reflection_color.array() * shoot_ray(scene,reflect,max_bounce).array(); 
}


Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &obj, const Intersection &hit, int max_bounce) {
    // Material for hit object
    const Material &mat = obj.material;

        
    // Ambient light contribution
    Vector3d ambient_color = obj.material.ambient_color.array() * scene.ambient_light.array();
    // Compute the color of the reflected ray and add its contribution to the current point color.
    Vector3d reflection_color(0, 0, 0);
    // Punctual lights contribution (direct lighting)
    Vector3d lights_color(0, 0, 0);
    // Compute the color of the refracted ray and add its contribution to the current point color.
    // Make sure to check for total internal reflection before shooting a new ray.
    Vector3d refraction_color(0, 0, 0);


    for (const Light &light : scene.lights) {
        Vector3d Li = (light.position - hit.position).normalized();
        Vector3d N = hit.normal;

        // Shoot a shadow ray to determine if the light should affect the intersection point
        Ray shadow_ray(hit.position + 0.001*Li, Li);
        if (!is_light_visible(scene,shadow_ray,light)) {
            continue;
        }

        set_object_color(ray, hit, lights_color, refraction_color, light, mat);
    }

    if (whichObject==4) //orb
        set_orb_reflection(scene, ray, hit, reflection_color, max_bounce, mat);
    

    // Rendering equation
    Vector3d C = ambient_color + lights_color + reflection_color + refraction_color;

    return C;
}

// -----------------------------------------------------------------------------

Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit) {
    // Find the object in the scene that intersects the ray first
    // The function must return 'nullptr' if no object is hit, otherwise it must
    // return a pointer to the hit object, and set both arguments 'hit' and
    // 'closest_t' to their expected values

    int closest_index = -1;
    closest_hit.ray_param = std::numeric_limits<double>::max();
    for (int i = 0; i < scene.objects.size(); ++i) {
        Intersection new_hit;
        if (scene.objects[i]->intersect(ray, new_hit)) {
            if (new_hit.ray_param < closest_hit.ray_param) {
                closest_index = i;
                closest_hit = new_hit;

                whichObject = i; //track which object we're on
            }
        }
    }
    if (closest_index < 0) {
        return nullptr;
    } else {
        return scene.objects[closest_index].get();
    }
}

bool is_light_visible(const Scene &scene, const Ray &ray, const Light &light) {
    Intersection hit;
    
    for (int i = 0; i < scene.objects.size(); ++i) {
        if (scene.objects[i]->intersect(ray, hit)) {
            return false;
        }
    }

    if (find_nearest_object(scene, ray, hit)) {
        double t = (ray.origin - light.position).norm();
        return t < hit.ray_param;
    }
    return true;
}

Vector3d shoot_ray(const Scene &scene, const Ray &ray, int max_bounce) {
    Intersection hit;

    if (Object * obj = find_nearest_object(scene, ray, hit)) {
        // 'obj' is not null and points to the object of the scene hit by the ray
        return ray_color(scene, ray, *obj, hit, max_bounce);
    } else {
        // 'obj' is null, we must return the background color
        for (const Light &light : scene.lights) {
            return scene.background_color;
        }               
    }
}

////////////////////////////////////////////////////////////////////////////////

void render_scene(const Scene &scene) {
    std::cout << "Simple ray tracer." << std::endl;

    int w = 960;
    int h = 720;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);
    double scale_y = scene.camera.focal_length * std::tan(scene.camera.field_of_view * 0.5);
    double scale_x = scale_y * aspect_ratio;

    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    // from the sensor, and is scaled from the canonical [-1,1] in order
    // to produce the target field of view.
    Vector3d grid_origin(-scale_x, scale_y, -scene.camera.focal_length);
    Vector3d x_displacement(2.0/w*scale_x, 0, 0);
    Vector3d y_displacement(0, -2.0/h*scale_y, 0);

    for (unsigned i = 0; i < w; ++i) {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Ray tracing: " << (100.0 * i) / w << "%\r" << std::flush;
        for (unsigned j = 0; j < h; ++j) {
            xPixel = i;
            yPixel = j;

            Vector3d shift = grid_origin + (i+0.5)*x_displacement + (j+0.5)*y_displacement;

            // Prepare the ray
            Ray ray;

            if (scene.camera.is_perspective) {
                // Perspective camera
                ray.origin = scene.camera.position;
                Vector3d target = scene.camera.position + shift;
                ray.direction = (target - ray.origin).normalized();
            } else {
                // Orthographic camera
                ray.origin = scene.camera.position + Vector3d(shift[0], shift[1], 0);
                ray.direction = Vector3d(0, 0, -1);
            }

            int max_bounce = 5;
            Vector3d C = shoot_ray(scene, ray, max_bounce);
            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = 1;
        }
    }

    std::cout << "Ray tracing: 100%  " << std::endl;

    // Save to png
    const std::string filename("raytrace.png");
    write_matrix_to_png(R, G, B, A, filename);
}


////////////////////////////////////////////////////////////////////////////////

Scene load_scene(const std::string &filename) {
    Scene scene;

    // Load json data from scene file
    json data;
    std::ifstream in(filename);
    in >> data;

    // Helper function to read a Vector3d from a json array
    auto read_vec3 = [] (const json &x) {
        return Vector3d(x[0], x[1], x[2]);
    };

    // Read scene info
    scene.ground = read_vec3(data["Scene"]["Ground"]);
    scene.background_color = read_vec3(data["Scene"]["Background"]);
    scene.ambient_light = read_vec3(data["Scene"]["Ambient"]);

    // Read camera info
    scene.camera.is_perspective = data["Camera"]["IsPerspective"];
    scene.camera.position = read_vec3(data["Camera"]["Position"]);
    scene.camera.field_of_view = data["Camera"]["FieldOfView"];
    scene.camera.focal_length = data["Camera"]["FocalLength"];
    scene.camera.lens_radius = data["Camera"]["LensRadius"];

    // Read materials
    for (const auto &entry : data["Materials"]) {
        Material mat;
        mat.ambient_color = read_vec3(entry["Ambient"]);
        mat.diffuse_color = read_vec3(entry["Diffuse"]);
        mat.specular_color = read_vec3(entry["Specular"]);
        mat.reflection_color = read_vec3(entry["Mirror"]);
        mat.refraction_color = read_vec3(entry["Refraction"]);
        mat.refraction_index = entry["RefractionIndex"];
        mat.specular_exponent = entry["Shininess"];
        scene.materials.push_back(mat);
    }

    // Read lights
    for (const auto &entry : data["Lights"]) {
        Light light;
        light.position = read_vec3(entry["Position"]);
        light.intensity = read_vec3(entry["Color"]);
        scene.lights.push_back(light);
    }

    // Read objects
    for (const auto &entry : data["Objects"]) {
        ObjectPtr object;
        if (entry["Type"] == "Sphere") {
            auto sphere = std::make_shared<Sphere>();
            sphere->position = read_vec3(entry["Position"]);
            sphere->radius = entry["Radius"];
            object = sphere;
        } else if (entry["Type"] == "Parallelogram") {
            // TODO
        } else if (entry["Type"] == "Mesh") {
            // Load mesh from a file
            std::string filename = std::string(DATA_DIR) + entry["Path"].get<std::string>();
            object = std::make_shared<Mesh>(filename);
        }
        object->material = scene.materials[entry["Material"]];
        scene.objects.push_back(object);
    }

    return scene;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " scene.json" << std::endl;
		return 1;
	}
	Scene scene = load_scene(argv[1]);
	render_scene(scene);
	return 0;
}
