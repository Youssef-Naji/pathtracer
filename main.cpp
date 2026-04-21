#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <cmath>
#include <random>
#include <omp.h>


#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#ifndef M_PI
#define M_PI 3.14159265358979323856
#endif

static std::default_random_engine engine[32];
static std::uniform_real_distribution<double> uniform(0, 1);

double sqr(double x) { return x * x; };

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
		data[0] /= n;
		data[1] /= n;
		data[2] /= n;
	}
	double operator[](int i) const { return data[i]; };
	double& operator[](int i) { return data[i]; };
	double data[3];
};

Vector operator+(const Vector& a, const Vector& b) {
	return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator-(const Vector& a, const Vector& b) {
	return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator*(const double a, const Vector& b) {
	return Vector(a*b[0], a*b[1], a*b[2]);
}
Vector operator*(const Vector& a, const double b) {
	return Vector(a[0]*b, a[1]*b, a[2]*b);
}
Vector operator/(const Vector& a, const double b) {
	return Vector(a[0] / b, a[1] / b, a[2] / b);
}
double dot(const Vector& a, const Vector& b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector cross(const Vector& a, const Vector& b) {
	return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}

class Ray {
public:
	Ray(const Vector& origin, const Vector& unit_direction) : O(origin), u(unit_direction) {};
	Vector O, u;
};

class Object {
public:
	Object(const Vector& albedo, bool mirror = false, bool transparent = false) : albedo(albedo), mirror(mirror), transparent(transparent) {};

	virtual bool intersect(const Ray& ray, Vector& P, double& t, Vector& N) const = 0;

	Vector albedo;
	bool mirror, transparent;
};

class Sphere : public Object {
public:
	Sphere(const Vector& center, double radius, const Vector& albedo, bool mirror = false, bool transparent = false) : ::Object(albedo, mirror, transparent), C(center), R(radius) {};

	// returns true iif there is an intersection between the ray and the sphere
	// if there is an intersection, also computes the point of intersection P, 
	// t>=0 the distance between the ray origin and P (i.e., the parameter along the ray)
	// and the unit normal N
	   // returns true iif there is an intersection between the ray and the sphere
   // if there is an intersection, also computes the point of intersection P,
   // t>=0 the distance between the ray origin and P (i.e., the parameter along the ray)
   // and the unit normal N
   	bool intersect(const Ray& ray, Vector& P, double &t, Vector& N) const {
        // TODO (lab 1) : compute the intersection (just true/false at the begining of lab 1, then P, t and N as well)
       	double b = dot(ray.u, ray.O - C);
       	double delta = sqr(b) - (dot(ray.O - C, ray.O - C) - R*R);
       	if (delta < 0) return false;
       	double t2 = -b - sqrt(delta);
       	double t1 = -b + sqrt(delta);
       	if (t2 > 0) {
           	t = t2;
       	} else if (t1 > 0) {
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

Sphere center_sphere(Vector(0, 0, -55), 10., Vector(0.8, 0.8, 0.8));

// I will provide you with an obj mesh loader (labs 3 and 4)
class TriangleMesh : public Object {
public:
	TriangleMesh(const Vector& albedo, bool mirror = false, bool transparent = false) : ::Object(albedo, mirror, transparent) {};

	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N) const {
		// TODO (labs 3 and 4)
		return false;
	}
};


class Scene {
public:
	Scene() {};
	void addObject(const Object* obj) {
		objects.push_back(obj);
	}

	// returns true iif there is an intersection between the ray and any object in the scene
    // if there is an intersection, also computes the point of the *nearest* intersection P, 
    // t>=0 the distance between the ray origin and P (i.e., the parameter along the ray)
    // and the unit normal N. 
	// Also returns the index of the object within the std::vector objects in object_id
	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N, int &object_id) const  {

		// TODO (lab 1): iterate through the objects and check the intersections with all of them, 
		// and keep the closest intersection, i.e., the one if smallest positive value of t
		bool has_intersection = false;
		t = 1e20;
		for (int i = 0; i < objects.size(); i++) {
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
		


	// return the radiance (color) along ray
Vector getColor(const Ray& ray, int recursion_depth) {








    if (recursion_depth >= max_light_bounce) return Vector(0, 0, 0);








    // TODO (lab 1) : if intersect with ray, use the returned information to compute the color ; otherwise black
    // in lab 1, the color only includes direct lighting with shadows
    Vector P, N;
    double t;
    int object_id;
    if (intersect(ray, P, t, N, object_id)) {








        if (objects[object_id]->mirror) {








             // return getColor in the reflected direction, with recursion_depth+1 (recursively)
        } // else








        if (objects[object_id]->transparent) { // optional








             // return getColor in the refraction direction, with recursion_depth+1 (recursively)
        } // else


         // test if there is a shadow by sending a new ray
         // if there is no shadow, compute the formula with dot products etc.
        Vector light_to_vector = light_position - P;
        Vector light_direction = light_to_vector / light_to_vector.norm();
        Ray shadow_ray(P + 1e-6 * N, light_direction);
        Vector shadow_P, shadow_N;
        double shadow_t;
        int shadow_id;


        Vector direct(0, 0, 0);
        if (intersect(shadow_ray, shadow_P, shadow_t, shadow_N, shadow_id)) {
            if (shadow_t < light_to_vector.norm() - 1e-6) {
                direct = Vector(0, 0, 0);
            } else {
                double attenuation = light_intensity / (4 * M_PI * light_to_vector.norm2());
                Vector material_color = objects[object_id]->albedo / M_PI;
                double solid_angle = std::max(0., dot(N, light_direction));
                direct = attenuation * material_color * solid_angle;
            }
        } else {
            double attenuation = light_intensity / (4 * M_PI * light_to_vector.norm2());
            Vector material_color = objects[object_id]->albedo / M_PI;
            double solid_angle = std::max(0., dot(N, light_direction));
            direct = attenuation * material_color * solid_angle;
        }


         // TODO (lab 2) : add indirect lighting component with a recursive call
        int tid = omp_get_thread_num();
        double r1 = uniform(engine[tid]);
        double r2 = uniform(engine[tid]);

        Vector u_dir;
        if (fabs(N[0]) <= fabs(N[1]) && fabs(N[0]) <= fabs(N[2]))
            u_dir = cross(N, Vector(1, 0, 0));
        else if (fabs(N[1]) <= fabs(N[2]))
            u_dir = cross(N, Vector(0, 1, 0));
        else
            u_dir = cross(N, Vector(0, 0, 1));
        u_dir.normalize();
        Vector v_dir = cross(N, u_dir);

        double cos_t = sqrt(1 - r2);
        double sin_t = sqrt(r2);
        double phi = 2 * M_PI * r1;
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
	int W = 512;
	int H = 512;

	for (int i = 0; i<32; i++) {
		engine[i].seed(i);
	}


	Sphere center_sphere(Vector(0, 0, 0), 10., Vector(0.8, 0.8, 0.8));

	
	Sphere wall_left(Vector(-1000, 0, 0), 940, Vector(0.5, 0.8, 0.1));
	Sphere wall_right(Vector(1000, 0, 0), 940, Vector(0.9, 0.2, 0.3));
	Sphere wall_front(Vector(0, 0, -1000), 940, Vector(0.1, 0.6, 0.7));
	Sphere wall_behind(Vector(0, 0, 1000), 940, Vector(0.8, 0.2, 0.9));
	Sphere ceiling(Vector(0, 1000, 0), 940, Vector(0.3, 0.5, 0.3));
	Sphere floor(Vector(0, -1000, 0), 990, Vector(0.6, 0.5, 0.7));

	Scene scene;
	scene.camera_center = Vector(0, 0, 55);
	scene.light_position = Vector(-10,20,40);
	scene.light_intensity = 3E7;
	scene.fov = 60 * M_PI / 180.;
	scene.gamma = 2.2;    // TODO (lab 1) : play with gamma ; typically, gamma = 2.2
	scene.max_light_bounce = 5;

	scene.addObject(&center_sphere);

	
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
        	Vector color;

        	// TODO (lab 1) : correct ray_direction so that it goes through each pixel (j, i)			
        	Vector ray_direction(j-(W/2)+0.5, (H/2)-i-0.5, -W/(2*tan(scene.fov/2)));
        	ray_direction.normalize();
        	Ray ray(scene.camera_center, ray_direction);

        
        	const int SPP = 100; 
        	color = Vector(0, 0, 0);
        	int thread_id = omp_get_thread_num();

        // TODO (lab 2) : add Monte Carlo / averaging of random ray contributions here
       
        	for (int s = 0; s < SPP; s++) {

            // TODO (lab 2) : add antialiasing by altering the ray_direction here
            
            	double u = uniform(engine[thread_id]);
            	double v = uniform(engine[thread_id]);

            	Vector ray_direction_aa(
                	j - (W/2) + u,
                	(H/2) - i - v,
               	 -W/(2*tan(scene.fov/2))
            	);
            	ray_direction_aa.normalize();
            	Ray ray_aa(scene.camera_center, ray_direction_aa);
            
            	color = color + scene.getColor(ray_aa, 0);
        	}
        	color = color / SPP;
        
        	image[(i * W + j) * 3 + 0] = std::min(255., std::max(0., 255. * std::pow(color[0] / 255., 1. / scene.gamma)));
        	image[(i * W + j) * 3 + 1] = std::min(255., std::max(0., 255. * std::pow(color[1] / 255., 1. / scene.gamma)));
        	image[(i * W + j) * 3 + 2] = std::min(255., std::max(0., 255. * std::pow(color[2] / 255., 1. / scene.gamma)));
    	}
	}
	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}