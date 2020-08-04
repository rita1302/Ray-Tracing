#include "pch.h"
#include "pixel.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"


#ifndef M_PI
#define M_PI (float)3.14159265358979323846
#endif

std::string outFileName = "../out.jpg";


class Ray {
public:
	vector_3d origin;
	vector_3d direction;

	Ray(const vector_3d &a, const vector_3d &b) : origin(a), direction(b) {}

	vector_3d expr(float x) const {
		return origin + direction * x;
	}
};


class Light {
public:
	vector_3d position;
	float intensity;

	Light(const vector_3d &p, const float &i) : position(p), intensity(i) {}
};


class Material {
public:
	vector_3d diffuse_color;
	vector_4d coef;
	float specular_exponent, ref_idx;

	Material(const float &r, const vector_4d coef, const vector_3d &color, const float &spec) : ref_idx(r), coef(coef), diffuse_color(color), specular_exponent(spec) {}
	Material() : coef(vector_4d(1, 0, 0, 0)), diffuse_color(), specular_exponent() {}
};


class Object {
public:
	Material material;
	Object(Material m) : material(m) {}
	virtual bool is_intersect(const Ray &ray, float &x, vector_3d &point, vector_3d &norm) const = 0;
};


class Sphere : public Object {
public:
	vector_3d center;
	float radius;

	Sphere(const vector_3d &c, const float &r, const Material &m) : Object(m), center(c), radius(r) {}

	bool is_intersect(const Ray &ray, float &x, vector_3d &point, vector_3d &norm) const { // пересекается ли луч со сферой
		vector_3d vect = ray.origin - center; // вектор от центра сферы до начала луча

		float first = ray.direction * ray.direction; 
		float second = vect * ray.direction;
		float third = vect * vect - radius * radius;
		float discr = second * second - first * third;
		if (discr < 0) return false;

		float sqrt_discr = sqrtf(discr);
		float x_1 = (-second - sqrt_discr) / first;
		float x_2 = (-second + sqrt_discr) / first;

		x = std::fmin(x_1, x_2);

		if (x < 0) return false;
		
		point = ray.expr(x);
		norm = (point - center).normalize();
		
		return true;
	}
};


class Plane : public Object {
public:
	vector_3d normal, one_point;

	Plane(const vector_3d &n, const vector_3d &p, Material m) : Object(m), normal(n.normalize()), one_point(p) {}

	bool is_intersect(const Ray &ray, float &x, vector_3d &point, vector_3d &norm) const {
		float eps = 1e-5;
		x = ((one_point - ray.origin) * normal) / (normal * ray.direction + eps);

		if (x < 0) return false;
		point = ray.expr(x);
		norm = normal;
		return true;
	}
};


class Ellipsoid : public Object {
public:
	float a, b, c;
	vector_3d center;

	Ellipsoid(const vector_3d &c, const float &x, const float &y, const float &z, Material m): Object(m), a(x), b(y), c(z) , center(c) {}

	bool is_intersect(const Ray &ray, float &x, vector_3d &point, vector_3d &norm) const {

		vector_3d coef = (vector_3d(1 / a, 1 / b, 1 / c)).sqr();
		vector_3d r_o = ray.origin;
		vector_3d r_d = ray.direction;

		vector_3d vect = r_o - center;

		float first = r_d.sqr() * coef;
		float second = vect.mul(r_d) * coef;
		float third = vect.sqr() * coef - 1;
		float discr = second * second - first * third;
		if (discr < 0) return false;

		float sqrt_discr = sqrtf(discr);
		float x_1 = (-second - sqrt_discr) / first;
		float x_2 = (-second + sqrt_discr) / first;

		x = std::fmin(x_1, x_2);

		if (x < 0) return false;

		point = ray.expr(x);
		vector_3d norm_1 = (point - center).mul(coef);
		norm = norm_1.normalize();

		return true;
	}
};


vector_3d refract(const vector_3d &inten, const vector_3d &norm, const float &ref_idx) { 

	float cos_angle = -std::fmax(-1.f, std::fmin(1.f, inten * norm));
	float etai = 1, etat = ref_idx;
	vector_3d n = norm;
	if (cos_angle < 0) { 
		cos_angle = -cos_angle;
		std::swap(etai, etat); 
		n = vector_3d(0,0,0) - norm;
	}
	float eta = etai / etat;
	float k = 1 - eta * eta*(1 - cos_angle * cos_angle);
	return k < 0 ? vector_3d(0, 0, 0) : inten * eta + n * (eta * cos_angle - sqrtf(k));
}


void save_image(const std::vector<vector_3d> &image, int width, int height, const char *outFile) {
	std::vector<unsigned char> STBimage(width * height * 3);
	for (int i = 0; i < height * width; ++i) {

		for (int j = 0; j < 3; ++j) {
			STBimage[i * 3 + j] = (unsigned char)(255.0f * std::fmax(0.f, std::fmin(image[i][j], 1.f)));
		}
	}
	stbi_write_jpg(outFile, width, height, 3, STBimage.data(), 100);
}


bool cast_ray_many(const Ray &ray, const std::vector<Object *> &spheres, vector_3d &point, vector_3d &norm, const Material **material) {
	vector_3d cur_point, cur_norm;
	float cur_dist, min_dist = std::numeric_limits<float>::max();
	for (const auto &sphere : spheres) {
		if (sphere->is_intersect(ray, cur_dist, cur_point, cur_norm) && cur_dist < min_dist) {
			point = cur_point;
			norm = cur_norm;
			*material = &sphere->material;
			min_dist = cur_dist;
		}
	}
	return min_dist < 100;
}


vector_3d cast_ray(const Ray &ray, const std::vector<Object *> &spheres, const std::vector<Light> &lights, int depth=0) {
	vector_3d point, norm;
	const Material *material = nullptr;

	float eps = 1e-3;

	if (depth > 2 || !cast_ray_many(ray, spheres, point, norm, &material)) {
		return vector_3d(0.5, 0.5, 0.5); 
	}

	vector_3d ray_dir = ray.direction;
	vector_3d r_direction = ray_dir.reflect(norm).normalize();
	vector_3d r_direction_1 = refract(ray_dir, norm, material->ref_idx).normalize();

	vector_3d r_origin = r_direction * norm < 0 ? point - norm * eps : point + norm * eps; 
	vector_3d r_origin_1 = r_direction_1 * norm < 0 ? point - norm * eps : point + norm * eps;

	Ray ray_2(r_origin, r_direction);
	Ray ray_3(r_origin_1, r_direction_1);


	vector_3d reflect_color(0, 0, 0);
	vector_3d refract_color(0, 0, 0);

	if (material->coef[2] != 0)
		reflect_color = cast_ray(ray_2, spheres, lights, depth + 1);

	if (material->coef[3] != 0)
		refract_color = cast_ray(ray_3, spheres, lights, depth + 1);


	float diffuse_light_intensity = 0;
	float specular_light_intensity = 0;

	for (int i = 0; i < lights.size(); i++) {
		vector_3d direction = (lights[i].position - point).normalize();
		float distance = (lights[i].position - point).point_norm();
		vector_3d shadow_origin = direction * norm < 0 ? point - norm * eps : point + norm * eps;
		vector_3d shadow_point, shadow_norm;
		const Material *new_material = nullptr;
		Ray ray_1(shadow_origin, direction);

		if (cast_ray_many(ray_1, spheres, shadow_point, shadow_norm, &new_material) && (shadow_point - shadow_origin).point_norm() < distance)
			continue;
		diffuse_light_intensity += lights[i].intensity * std::fmax(0.f, direction * norm);
		specular_light_intensity += powf(std::fmax(0.f, direction.reflect(norm) * ray.direction), material->specular_exponent) * lights[i].intensity;
	}

	return material->diffuse_color * diffuse_light_intensity * material->coef[0] + specular_light_intensity * material->coef[1] + reflect_color * material->coef[2] + refract_color
		* material->coef[3];
}


void render(const std::vector<Object *> &objects, const std::vector<Light> &lights) { // сохранение изображения
	const int width = 512; 
	const int height = 512; 
	const float fov = M_PI / 3.f;
	std::vector<vector_3d> image(width * height);
	bool remove = true;

	for (int j = 0; j < height; j++) { // по столбцам
		for (int i = 0; i < width; i++) { // по строкам
			vector_3d temp(i, j, 0);
			vector_3d start(0, 0, 0);
			image[j * width + i] = cast_ray(Ray(start, temp.transform(width, height, fov)), objects, lights);
			if (remove) {
				const float bias = 0.5f;

				vector_3d temp_1((float)i + bias, j, 0);
				image[j * width + i] += cast_ray(Ray(start, temp_1.transform(width, height, fov)), objects, lights);
				vector_3d temp_2((float)i - bias, j, 0);
				image[j * width + i] += cast_ray(Ray(start, temp_2.transform(width, height, fov)), objects, lights);
				vector_3d temp_3(i, (float)j + bias, 0);
				image[j * width + i] += cast_ray(Ray(start, temp_3.transform(width, height, fov)), objects, lights);
				vector_3d temp_4(i, (float)j - bias, 0);
				image[j * width + i] += cast_ray(Ray(start, temp_4.transform(width, height, fov)), objects, lights);

				image[j * width + i] /= 5.0f;
			}
		}
	}

	save_image(image, width, height, outFileName.c_str());
}


int main(int argc, char **argv) {
    std::unordered_map<std::string, std::string> cmdLineParams;
    for (uint i = 0; i < argc; ++i) {
        std::string key(argv[i]);
        if(!key.empty() && (key[0] == '-')) {
            if (i != argc - 1) {
                cmdLineParams[key] = argv[i + 1];
                i++;
            } else {
                cmdLineParams[key] = "";
            }
        }
    }
    if (cmdLineParams.find("-out") != cmdLineParams.end()) {
        outFileName = cmdLineParams["-out"];
    }

	std::vector<Object *> objects;
	std::vector<Light>  lights;

	lights.push_back(Light(vector_3d(-20, 20, 20), 1.5));
	lights.push_back(Light(vector_3d(30, 50, -25), 1.8));
	lights.push_back(Light(vector_3d(30, 20, 30), 1.7));

	Material glass(1.5, vector_4d(0.0, 0.5, 0.1, 0.8), vector_3d(0.6, 0.7, 0.8), 125.); // материал стекло
	Material purple(1.0, vector_4d(0.9, 0.1, 0.0, 0.0), vector_3d(0.3, 0, 0.5), 10.);
	Material mirror(1.0, vector_4d(0.0, 10.0, 0.8, 0.0), vector_3d(1.0, 1.0, 1.0), 1425.); // материал зеркало
	Material black(1.0, vector_4d(0.9, 0.1, 0.0, 0.0), vector_3d(0, 0, 0), 10);
	Material cheese(1.0, vector_4d(0.9, 0.1, 0.0, 0.0), vector_3d(1, 0.5, 0), 50);

	//центральная мышь с зеркальным шаром

	objects.push_back(new Plane(vector_3d(0, 1, 0), vector_3d(0, -1, 0), cheese));
	objects.push_back(new Sphere(vector_3d(-1, -0.5, -18), 3, glass));
	objects.push_back(new Sphere(vector_3d(-0.3, 0.3, -18), 0.3, black));
	objects.push_back(new Sphere(vector_3d(-1.7, 0.3, -18), 0.3, black));
	objects.push_back(new Sphere(vector_3d(-1, -0.8, -18), 0.3, black));
	objects.push_back(new Sphere(vector_3d(-3.5, 2.5, -18), 1, black));
	objects.push_back(new Sphere(vector_3d(1.5, 2.5, -18), 1, black));
	objects.push_back(new Sphere(vector_3d(7, 5, -18), 4, mirror));

	//правая и левая мыши

	objects.push_back(new Sphere(vector_3d(-2, -0.7, -5), 0.5, glass));
	objects.push_back(new Sphere(vector_3d(-2.15, -0.6, -5), 0.07, black));
	objects.push_back(new Sphere(vector_3d(-1.85, -0.6, -5), 0.08, black));
	objects.push_back(new Sphere(vector_3d(-2, -0.85, -5), 0.07, black));
	objects.push_back(new Sphere(vector_3d(-2.5, -0.2, -5), 0.19, black));
	objects.push_back(new Sphere(vector_3d(-1.5, -0.2, -5), 0.2, black));

	objects.push_back(new Sphere(vector_3d(2, -0.7, -5), 0.5, glass));
	objects.push_back(new Sphere(vector_3d(2.15, -0.6, -5), 0.07, black));
	objects.push_back(new Sphere(vector_3d(1.85, -0.6, -5), 0.08, black));
	objects.push_back(new Sphere(vector_3d(2, -0.85, -5), 0.07, black));
	objects.push_back(new Sphere(vector_3d(2.5, -0.2, -5), 0.19, black));
	objects.push_back(new Sphere(vector_3d(1.5, -0.2, -5), 0.2, black));

	//надпись из эллипсоидов

	objects.push_back(new Ellipsoid(vector_3d(-9, 7, -18), 0.5, 3.5, 0.5, purple));
	objects.push_back(new Ellipsoid(vector_3d(-6, 7, -18), 0.5, 3.5, 0.5, purple));
	objects.push_back(new Ellipsoid(vector_3d(-7.5, 7, -18), 1.9, 0.3, 0.3, purple));
	objects.push_back(new Ellipsoid(vector_3d(-3, 7, -18), 0.5, 3.5, 0.5, purple));
	objects.push_back(new Ellipsoid(vector_3d(0, 7.6, -18), 0.4, 2.8, 0.4, purple));
	objects.push_back(new Sphere(vector_3d(0, 4, -18), 0.4, purple));

	render(objects, lights);

	return 0;
}
