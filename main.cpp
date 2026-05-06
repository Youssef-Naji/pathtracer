#define _CRT_SECURE_NO_WARNINGS 1

#include <vector>
#include <cmath>
#include <random>
#include <omp.h>
#include <map>
#include <string>
#include <fstream>
#include <algorithm>
#include <iostream>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#ifndef M_PI
#define M_PI 3.14159265358979323856
#endif

static std::default_random_engine engine[32];
static std::uniform_real_distribution<double> uniform(0, 1);

double sqr(double x) { return x * x; }

class Vector {
public:
	explicit Vector(double x = 0, double y = 0, double z = 0) {
		data[0] = x;
		data[1] = y;
		data[2] = z;
	}

	double norm2() const {
		return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
	}

	double norm() const {
		return sqrt(norm2());
	}

	void normalize() {
		double n = norm();
		if (n > 0) {
			data[0] /= n;
			data[1] /= n;
			data[2] /= n;
		}
	}

	double operator[](int i) const { return data[i]; }
	double& operator[](int i) { return data[i]; }

	double data[3];
};

Vector operator+(const Vector& a, const Vector& b) {
	return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}

Vector operator-(const Vector& a, const Vector& b) {
	return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

Vector operator*(const double a, const Vector& b) {
	return Vector(a * b[0], a * b[1], a * b[2]);
}

Vector operator*(const Vector& a, const double b) {
	return Vector(a[0] * b, a[1] * b, a[2] * b);
}

Vector operator/(const Vector& a, const double b) {
	return Vector(a[0] / b, a[1] / b, a[2] / b);
}

double dot(const Vector& a, const Vector& b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vector cross(const Vector& a, const Vector& b) {
	return Vector(
		a[1] * b[2] - a[2] * b[1],
		a[2] * b[0] - a[0] * b[2],
		a[0] * b[1] - a[1] * b[0]
	);
}

class Ray {
public:
	Ray(const Vector& origin, const Vector& unit_direction) : O(origin), u(unit_direction) {}
	Vector O, u;
};

class Object {
public:
	Object(const Vector& albedo, bool mirror = false, bool transparent = false)
		: albedo(albedo), mirror(mirror), transparent(transparent) {}

	virtual bool intersect(const Ray& ray, Vector& P, double& t, Vector& N) const = 0;

	Vector albedo;
	bool mirror, transparent;
};

class Sphere : public Object {
public:
	Sphere(const Vector& center, double radius, const Vector& albedo, bool mirror = false, bool transparent = false)
		: Object(albedo, mirror, transparent), C(center), R(radius) {}

	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N) const override {
		// TODO (lab 1) : compute the intersection (just true/false at the begining of lab 1, then P, t and N as well)
		double b = dot(ray.u, ray.O - C);
		double delta = sqr(b) - (dot(ray.O - C, ray.O - C) - R * R);

		if (delta < 0) return false;

		double t2 = -b - sqrt(delta);
		double t1 = -b + sqrt(delta);

		if (t2 > 1e-6) {
			t = t2;
		} else if (t1 > 1e-6) {
			t = t1;
		} else {
			return false;
		}

		P = ray.O + t * ray.u;
		N = P - C;
		N.normalize();
		return true;
	}

	double R;
	Vector C;
};

class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1,
		int ni = -1, int nj = -1, int nk = -1,
		int uvi = -1, int uvj = -1, int uvk = -1,
		int group = -1) {
		vtx[0] = vtxi; vtx[1] = vtxj; vtx[2] = vtxk;
		uv[0] = uvi; uv[1] = uvj; uv[2] = uvk;
		n[0] = ni; n[1] = nj; n[2] = nk;
		this->group = group;
	}

	int vtx[3];
	int uv[3];
	int n[3];
	int group;
};

class TriangleMesh : public Object {
public:
	TriangleMesh(const Vector& albedo, bool mirror = false, bool transparent = false)
		: Object(albedo, mirror, transparent) {}

	class BVHNode {
	public:
		Vector bbox_min;
		Vector bbox_max;
		int left = -1;
		int right = -1;
		int start = 0;
		int end = 0;
		bool leaf = false;
	};

	std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;
	std::vector<BVHNode> bvh;

	// first scale and then translate the current object
	void scale_translate(double s, const Vector& t) {
		for (int i = 0; i < (int)vertices.size(); i++) {
			vertices[i] = vertices[i] * s + t;
		}
	}

	// read an .obj file
	void readOBJ(const char* obj) {
		std::ifstream f(obj);
		if (!f) {
			std::cerr << "ERROR: cannot open OBJ file: " << obj << std::endl;
			return;
		}

		std::map<std::string, int> mtls;
		int curGroup = -1, maxGroup = -1;

		auto resolveIdx = [](int i, int size) {
			return i < 0 ? size + i : i - 1;
		};

		auto setFaceVerts = [&](TriangleIndices& t, int i0, int i1, int i2) {
			t.vtx[0] = resolveIdx(i0, (int)vertices.size());
			t.vtx[1] = resolveIdx(i1, (int)vertices.size());
			t.vtx[2] = resolveIdx(i2, (int)vertices.size());
		};

		auto setFaceUVs = [&](TriangleIndices& t, int j0, int j1, int j2) {
			t.uv[0] = resolveIdx(j0, (int)uvs.size());
			t.uv[1] = resolveIdx(j1, (int)uvs.size());
			t.uv[2] = resolveIdx(j2, (int)uvs.size());
		};

		auto setFaceNormals = [&](TriangleIndices& t, int k0, int k1, int k2) {
			t.n[0] = resolveIdx(k0, (int)normals.size());
			t.n[1] = resolveIdx(k1, (int)normals.size());
			t.n[2] = resolveIdx(k2, (int)normals.size());
		};

		std::string line;
		while (std::getline(f, line)) {
			line.erase(line.find_last_not_of(" \r\t\n") + 1);
			if (line.empty()) continue;

			const char* s = line.c_str();

			if (line.rfind("usemtl ", 0) == 0) {
				std::string matname = line.substr(7);
				auto result = mtls.emplace(matname, maxGroup + 1);
				if (result.second) curGroup = ++maxGroup;
				else curGroup = result.first->second;
			} else if (line.rfind("vn ", 0) == 0) {
				Vector v;
				sscanf(s, "vn %lf %lf %lf", &v[0], &v[1], &v[2]);
				v.normalize();
				normals.push_back(v);
			} else if (line.rfind("vt ", 0) == 0) {
				Vector v;
				sscanf(s, "vt %lf %lf", &v[0], &v[1]);
				uvs.push_back(v);
			} else if (line.rfind("v ", 0) == 0) {
				Vector pos, col;
				if (sscanf(s, "v %lf %lf %lf %lf %lf %lf", &pos[0], &pos[1], &pos[2], &col[0], &col[1], &col[2]) == 6) {
					for (int i = 0; i < 3; i++) col[i] = std::min(1.0, std::max(0.0, col[i]));
					vertexcolors.push_back(col);
				} else {
					sscanf(s, "v %lf %lf %lf", &pos[0], &pos[1], &pos[2]);
				}
				vertices.push_back(pos);
			} else if (line[0] == 'f') {
				int i[4], j[4], k[4], offset, nn;
				const char* cur = s + 1;
				TriangleIndices t;
				t.group = curGroup;

				if ((nn = sscanf(cur, "%d/%d/%d %d/%d/%d %d/%d/%d%n", &i[0], &j[0], &k[0], &i[1], &j[1], &k[1], &i[2], &j[2], &k[2], &offset)) == 9) {
					setFaceVerts(t, i[0], i[1], i[2]);
					setFaceUVs(t, j[0], j[1], j[2]);
					setFaceNormals(t, k[0], k[1], k[2]);
				} else if ((nn = sscanf(cur, "%d/%d %d/%d %d/%d%n", &i[0], &j[0], &i[1], &j[1], &i[2], &j[2], &offset)) == 6) {
					setFaceVerts(t, i[0], i[1], i[2]);
					setFaceUVs(t, j[0], j[1], j[2]);
				} else if ((nn = sscanf(cur, "%d//%d %d//%d %d//%d%n", &i[0], &k[0], &i[1], &k[1], &i[2], &k[2], &offset)) == 6) {
					setFaceVerts(t, i[0], i[1], i[2]);
					setFaceNormals(t, k[0], k[1], k[2]);
				} else if ((nn = sscanf(cur, "%d %d %d%n", &i[0], &i[1], &i[2], &offset)) == 3) {
					setFaceVerts(t, i[0], i[1], i[2]);
				} else {
					continue;
				}

				indices.push_back(t);
				cur += offset;

				while (*cur && *cur != '\n') {
					TriangleIndices t2;
					t2.group = curGroup;

					if ((nn = sscanf(cur, " %d/%d/%d%n", &i[3], &j[3], &k[3], &offset)) == 3) {
						setFaceVerts(t2, i[0], i[2], i[3]);
						setFaceUVs(t2, j[0], j[2], j[3]);
						setFaceNormals(t2, k[0], k[2], k[3]);
					} else if ((nn = sscanf(cur, " %d/%d%n", &i[3], &j[3], &offset)) == 2) {
						setFaceVerts(t2, i[0], i[2], i[3]);
						setFaceUVs(t2, j[0], j[2], j[3]);
					} else if ((nn = sscanf(cur, " %d//%d%n", &i[3], &k[3], &offset)) == 2) {
						setFaceVerts(t2, i[0], i[2], i[3]);
						setFaceNormals(t2, k[0], k[2], k[3]);
					} else if ((nn = sscanf(cur, " %d%n", &i[3], &offset)) == 1) {
						setFaceVerts(t2, i[0], i[2], i[3]);
					} else {
						cur++;
						continue;
					}

					indices.push_back(t2);
					cur += offset;
					i[2] = i[3];
					j[2] = j[3];
					k[2] = k[3];
				}
			}
		}
	}

	Vector triangle_centroid(int i) const {
		const TriangleIndices& tri = indices[i];
		return (vertices[tri.vtx[0]] + vertices[tri.vtx[1]] + vertices[tri.vtx[2]]) / 3.0;
	}

	void triangle_bbox(int i, Vector& bmin, Vector& bmax) const {
		const TriangleIndices& tri = indices[i];
		bmin = vertices[tri.vtx[0]];
		bmax = vertices[tri.vtx[0]];

		for (int a = 1; a < 3; a++) {
			const Vector& v = vertices[tri.vtx[a]];
			for (int k = 0; k < 3; k++) {
				bmin[k] = std::min(bmin[k], v[k]);
				bmax[k] = std::max(bmax[k], v[k]);
			}
		}
	}

	bool bbox_intersect(const Ray& ray, const Vector& bmin, const Vector& bmax, double max_t) const {
		double tmin = 0.0;
		double tmax = max_t;

		for (int k = 0; k < 3; k++) {
			if (fabs(ray.u[k]) < 1e-12) {
				if (ray.O[k] < bmin[k] || ray.O[k] > bmax[k]) return false;
			} else {
				double t1 = (bmin[k] - ray.O[k]) / ray.u[k];
				double t2 = (bmax[k] - ray.O[k]) / ray.u[k];

				if (t1 > t2) std::swap(t1, t2);

				tmin = std::max(tmin, t1);
				tmax = std::min(tmax, t2);

				if (tmin > tmax) return false;
			}
		}

		return true;
	}

	int build_bvh_recursive(int start, int end) {
		BVHNode node;
		node.start = start;
		node.end = end;

		Vector bmin(1e20, 1e20, 1e20);
		Vector bmax(-1e20, -1e20, -1e20);

		for (int i = start; i < end; i++) {
			Vector tri_min, tri_max;
			triangle_bbox(i, tri_min, tri_max);

			for (int k = 0; k < 3; k++) {
				bmin[k] = std::min(bmin[k], tri_min[k]);
				bmax[k] = std::max(bmax[k], tri_max[k]);
			}
		}

		node.bbox_min = bmin;
		node.bbox_max = bmax;

		int node_id = (int)bvh.size();
		bvh.push_back(node);

		int count = end - start;
		if (count <= 8) {
			bvh[node_id].leaf = true;
			return node_id;
		}

		Vector cmin(1e20, 1e20, 1e20);
		Vector cmax(-1e20, -1e20, -1e20);

		for (int i = start; i < end; i++) {
			Vector c = triangle_centroid(i);
			for (int k = 0; k < 3; k++) {
				cmin[k] = std::min(cmin[k], c[k]);
				cmax[k] = std::max(cmax[k], c[k]);
			}
		}

		Vector extent = cmax - cmin;
		int axis = 0;
		if (extent[1] > extent[axis]) axis = 1;
		if (extent[2] > extent[axis]) axis = 2;

		int mid = (start + end) / 2;

		std::nth_element(indices.begin() + start, indices.begin() + mid, indices.begin() + end,
			[&](const TriangleIndices& a, const TriangleIndices& b) {
				Vector ca = (vertices[a.vtx[0]] + vertices[a.vtx[1]] + vertices[a.vtx[2]]) / 3.0;
				Vector cb = (vertices[b.vtx[0]] + vertices[b.vtx[1]] + vertices[b.vtx[2]]) / 3.0;
				return ca[axis] < cb[axis];
			}
		);

		bvh[node_id].left = build_bvh_recursive(start, mid);
		bvh[node_id].right = build_bvh_recursive(mid, end);

		return node_id;
	}

	void build_bvh() {
		bvh.clear();
		if (!indices.empty()) {
			build_bvh_recursive(0, (int)indices.size());
		}
	}

	bool intersect_triangle(int idx, const Ray& ray, Vector& P, double& t, Vector& N) const {
		const TriangleIndices& tri = indices[idx];

		const Vector& A = vertices[tri.vtx[0]];
		const Vector& B = vertices[tri.vtx[1]];
		const Vector& C = vertices[tri.vtx[2]];

		Vector edge1 = B - A;
		Vector edge2 = C - A;

		Vector h = cross(ray.u, edge2);
		double a = dot(edge1, h);

		if (fabs(a) < 1e-8) return false;

		double f = 1.0 / a;
		Vector s = ray.O - A;

		double u = f * dot(s, h);
		if (u < 0.0 || u > 1.0) return false;

		Vector q = cross(s, edge1);

		double v = f * dot(ray.u, q);
		if (v < 0.0 || u + v > 1.0) return false;

		double t_temp = f * dot(edge2, q);
		if (t_temp <= 1e-8 || t_temp >= t) return false;

		t = t_temp;
		P = ray.O + ray.u * t;

		if (tri.n[0] >= 0 && tri.n[1] >= 0 && tri.n[2] >= 0 &&
			tri.n[0] < (int)normals.size() && tri.n[1] < (int)normals.size() && tri.n[2] < (int)normals.size()) {
			N = (1.0 - u - v) * normals[tri.n[0]] + u * normals[tri.n[1]] + v * normals[tri.n[2]];
			N.normalize();
		} else {
			N = cross(edge1, edge2);
			N.normalize();
		}

		if (dot(N, ray.u) > 0) {
			N = -1.0 * N;
		}

		return true;
	}

	bool intersect_bvh(int node_id, const Ray& ray, Vector& P, double& t, Vector& N) const {
		const BVHNode& node = bvh[node_id];

		if (!bbox_intersect(ray, node.bbox_min, node.bbox_max, t)) {
			return false;
		}

		bool hit = false;

		if (node.leaf) {
			for (int i = node.start; i < node.end; i++) {
				if (intersect_triangle(i, ray, P, t, N)) {
					hit = true;
				}
			}
		} else {
			bool hit_left = intersect_bvh(node.left, ray, P, t, N);
			bool hit_right = intersect_bvh(node.right, ray, P, t, N);
			hit = hit_left || hit_right;
		}

		return hit;
	}

	// TODO ray-mesh intersection (labs 3 and 4)
	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N) const override {
		// lab 4 : recursively apply the bounding-box test from a BVH datastructure
		if (bvh.empty()) return false;

		t = 1e20;
		return intersect_bvh(0, ray, P, t, N);
	}
};

class Scene {
public:
	Scene() {}

	void addObject(const Object* obj) {
		objects.push_back(obj);
	}

	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N, int& object_id) const {
		// TODO (lab 1): iterate through the objects and check the intersections with all of them,
		// and keep the closest intersection, i.e., the one if smallest positive value of t
		bool has_intersection = false;
		t = 1e20;

		for (int i = 0; i < (int)objects.size(); i++) {
			Vector Ptemp, Ntemp;
			double ttemp;

			if (objects[i]->intersect(ray, Ptemp, ttemp, Ntemp)) {
				if (ttemp < t) {
					has_intersection = true;
					t = ttemp;
					P = Ptemp;
					N = Ntemp;
					object_id = i;
				}
			}
		}

		return has_intersection;
	}

	Vector getColor(const Ray& ray, int recursion_depth) {
		if (recursion_depth >= max_light_bounce) return Vector(0, 0, 0);

		Vector P, N;
		double t;
		int object_id = -1;

		if (intersect(ray, P, t, N, object_id)) {
			if (objects[object_id]->mirror) {
				Vector reflected_direction = ray.u - 2.0 * dot(ray.u, N) * N;
				reflected_direction.normalize();
				return getColor(Ray(P + 1e-6 * N, reflected_direction), recursion_depth + 1);
			}

			Vector light_to_vector = light_position - P;
			Vector light_direction = light_to_vector / light_to_vector.norm();
			Ray shadow_ray(P + 1e-6 * N, light_direction);

			Vector shadow_P, shadow_N;
			double shadow_t;
			int shadow_id;

			Vector direct(0, 0, 0);
			double dist_to_light = light_to_vector.norm();

			bool shadow_hit = intersect(shadow_ray, shadow_P, shadow_t, shadow_N, shadow_id);
			bool in_shadow = shadow_hit && shadow_t < dist_to_light - 1e-6;

			if (!in_shadow) {
				double attenuation = light_intensity / (4.0 * M_PI * light_to_vector.norm2());
				Vector material_color = objects[object_id]->albedo / M_PI;
				double solid_angle = std::max(0.0, dot(N, light_direction));
				direct = attenuation * material_color * solid_angle;
			}

			// TODO (lab 2) : add indirect lighting component with a recursive call
			int tid = omp_get_thread_num();
			double r1 = uniform(engine[tid]);
			double r2 = uniform(engine[tid]);

			Vector u_dir;
			if (fabs(N[0]) <= fabs(N[1]) && fabs(N[0]) <= fabs(N[2])) {
				u_dir = cross(N, Vector(1, 0, 0));
			} else if (fabs(N[1]) <= fabs(N[2])) {
				u_dir = cross(N, Vector(0, 1, 0));
			} else {
				u_dir = cross(N, Vector(0, 0, 1));
			}

			u_dir.normalize();
			Vector v_dir = cross(N, u_dir);

			double cos_t = sqrt(1.0 - r2);
			double sin_t = sqrt(r2);
			double phi = 2.0 * M_PI * r1;

			Vector indirect_dir = sin_t * cos(phi) * u_dir + sin_t * sin(phi) * v_dir + cos_t * N;
			indirect_dir.normalize();

			Ray indirect_ray(P + 1e-6 * N, indirect_dir);
			Vector indirect = getColor(indirect_ray, recursion_depth + 1);

			indirect = Vector(
				objects[object_id]->albedo[0] * indirect[0],
				objects[object_id]->albedo[1] * indirect[1],
				objects[object_id]->albedo[2] * indirect[2]
			);

			return direct + indirect;
		}

		return Vector(0, 0, 0);
	}

	std::vector<const Object*> objects;
	Vector camera_center, light_position;
	double fov, gamma, light_intensity;
	int max_light_bounce;
};

int main() {
	int W = 1000;
	int H = 1000;

	for (int i = 0; i < 32; i++) {
		engine[i].seed(i);
	}

	TriangleMesh cat(Vector(0.8, 0.8, 0.8));
	cat.readOBJ("Models_F0202A090/cat.obj");

	std::cerr << "cat vertices = " << cat.vertices.size() << std::endl;
	std::cerr << "cat triangles = " << cat.indices.size() << std::endl;

	cat.scale_translate(0.8, Vector(0, -10, -35));
	cat.build_bvh();

	std::cerr << "BVH nodes = " << cat.bvh.size() << std::endl;

	Sphere center_sphere(Vector(0, 0, 0), 10.0, Vector(0.8, 0.8, 0.8));

	Sphere wall_left(Vector(-1000, 0, 0), 940, Vector(0.5, 0.8, 0.1));
	Sphere wall_right(Vector(1000, 0, 0), 940, Vector(0.9, 0.2, 0.3));
	Sphere wall_front(Vector(0, 0, -1000), 940, Vector(0.1, 0.6, 0.7));
	Sphere wall_behind(Vector(0, 0, 1000), 940, Vector(0.8, 0.2, 0.9));
	Sphere ceiling(Vector(0, 1000, 0), 940, Vector(0.3, 0.5, 0.3));
	Sphere floor(Vector(0, -1000, 0), 990, Vector(0.6, 0.5, 0.7));

	Scene scene;
	scene.camera_center = Vector(0, 0, 55);
	scene.light_position = Vector(-10, 20, 40);
	scene.light_intensity = 3E7;
	scene.fov = 60 * M_PI / 180.0;
	scene.gamma = 2.2;
	scene.max_light_bounce = 5;

	// Keep this commented if you want to see only the cat.
	// scene.addObject(&center_sphere);

	scene.addObject(&cat);
	scene.addObject(&wall_left);
	scene.addObject(&wall_right);
	scene.addObject(&wall_front);
	scene.addObject(&wall_behind);
	scene.addObject(&ceiling);
	scene.addObject(&floor);

	std::vector<unsigned char> image(W * H * 3, 0);

#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {
			Vector color(0, 0, 0);

			const int SPP = 32;
			int thread_id = omp_get_thread_num();

			// TODO (lab 2) : add Monte Carlo / averaging of random ray contributions here
			for (int s = 0; s < SPP; s++) {
				// TODO (lab 2) : add antialiasing by altering the ray_direction here
				double u = uniform(engine[thread_id]);
				double v = uniform(engine[thread_id]);

				Vector ray_direction(
					j - (W / 2.0) + u,
					(H / 2.0) - i - v,
					-W / (2.0 * tan(scene.fov / 2.0))
				);

				ray_direction.normalize();
				Ray ray(scene.camera_center, ray_direction);

				color = color + scene.getColor(ray, 0);
			}

			color = color / SPP;

			image[(i * W + j) * 3 + 0] = std::min(255.0, std::max(0.0, 255.0 * std::pow(color[0] / 255.0, 1.0 / scene.gamma)));
			image[(i * W + j) * 3 + 1] = std::min(255.0, std::max(0.0, 255.0 * std::pow(color[1] / 255.0, 1.0 / scene.gamma)));
			image[(i * W + j) * 3 + 2] = std::min(255.0, std::max(0.0, 255.0 * std::pow(color[2] / 255.0, 1.0 / scene.gamma)));
		}
	}

	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}
