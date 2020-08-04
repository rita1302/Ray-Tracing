#pragma once

//работа с трехмерными и четырехмерными векторами
class vector_3d {
public:
	float x, y, z;
	vector_3d() : x(0), y(0), z(0) {}
	vector_3d(float a, float b, float c) : x(a), y(b), z(c) {}

	float operator*(const vector_3d &point) const {
		return point.x * x + point.y * y + point.z * z;
	}

	vector_3d operator-(const vector_3d &point) const {
		return vector_3d(x - point.x, y - point.y, z - point.z);
	}

	float  operator [] (int i) const {
		if (i == 0) return x;
		if (i == 1) return y;
		if (i == 2) return z;
	}

	vector_3d operator*(float a) const {
		return vector_3d(a * x, a * y, a * z);
	}

	vector_3d operator+(const vector_3d &point) const {
		return vector_3d(x + point.x, y + point.y, z + point.z);
	}

	vector_3d& operator+=(const vector_3d &point){
		x += point.x;
		y += point.y;
		z += point.z;
		return *this;
	}


	vector_3d normalize() const{
		float norm = sqrtf(x * x + y * y + z * z);
		return vector_3d(x / norm, y / norm, z / norm);
	}

	float point_norm() {
		float norm = sqrtf(x * x + y * y + z * z);
		return norm;
	}

	vector_3d transform(float width, float height, float fov) {

		float u = x - width / 2.0f;
		float v = -(y - height / 2.0f);
		float w = -(height / 2.0f) / tanf(fov / 2.0f);

		return vector_3d(u, v, w).normalize();
	}

	vector_3d reflect(const vector_3d &norm) {
		return *this - norm * 2.f * (*this * norm);
	}

	vector_3d operator+(float a) {
		return vector_3d(x + a, y + a, z + a);
	}

	vector_3d sqr() const{
		return vector_3d(x * x, y * y, z * z);
	}

	vector_3d mul(vector_3d point) const{
		return vector_3d(x * point.x, y * point.y, z * point.z);
	}

	vector_3d& operator/=(const float &point) {
		x /= point;
		y /= point;
		z /= point;
		return *this;
	}
};

class vector_4d {
public:
	float x, y, z, k;
	vector_4d() : x(0), y(0), z(0), k(0) {}
	vector_4d(float a, float b, float c, float d) : x(a), y(b), z(c), k(d) {}

	float  operator [] (int i) const {
		if (i == 0) return x;
		if (i == 1) return y;
		if (i == 2) return z;
		if (i == 3) return k;
	}
};