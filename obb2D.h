#pragma once
#include<math.h>


class Vector2
{
public:
	typedef float data_type;

	Vector2() {
		m_Data[0] = m_Data[1] = 0.0f;
	}

	Vector2(const Vector2& other) {
		m_Data[0] = other.m_Data[0];
		m_Data[1] = other.m_Data[1];
	}

	Vector2(data_type x, data_type y) {
		m_Data[0] = x;
		m_Data[1] = y;
	}

	double dot(const Vector2& other) const {
		return other.m_Data[0] * m_Data[0] + other.m_Data[1] * m_Data[1];
	}

	double squaredLength() const {
		return sqrtf(m_Data[0] * m_Data[0] + m_Data[1] * m_Data[1]);
	}

	Vector2& operator/(data_type factor) {
		m_Data[0] /= factor;
		m_Data[1] /= factor;
		return *this;
	}

	Vector2& operator/=(data_type factor) {
		m_Data[0] /= factor;
		m_Data[1] /= factor;
		return *this;
	}

	Vector2& operator*(data_type factor) {
		m_Data[0] *= factor;
		m_Data[1] *= factor;
		return *this;
	}

	Vector2& operator*=(data_type factor) {
		m_Data[0] *= factor;
		m_Data[1] *= factor;
		return *this;
	}

	Vector2& operator+=(const Vector2& other) {
		m_Data[0] += other.m_Data[0];
		m_Data[1] += other.m_Data[1];
		return *this;
	}

	Vector2& operator=(const Vector2& other) {
		if (this == &other) {
			return *this;
		}
		m_Data[0] = other.m_Data[0];
		m_Data[1] = other.m_Data[1];
		return *this;
	}

	operator const data_type* () const {
		return &(m_Data[0]);
	}

	const data_type* data() const {
		return &(m_Data[0]);
	}

	friend static Vector2 operator-(const Vector2& first, const Vector2& second) {
		data_type x = first.m_Data[0] - second.m_Data[0];
		data_type y = first.m_Data[1] - second.m_Data[1];
		return Vector2(x, y);
	}

	friend static Vector2 operator+(const Vector2& first, const Vector2& second) {
		data_type x = first.m_Data[0] + second.m_Data[0];
		data_type y = first.m_Data[1] + second.m_Data[1];
		return Vector2(x, y);
	}

private:
	data_type m_Data[2];

};

class OBB2D {
private:
	/** Corners of the box, where 0 is the lower left. */
	Vector2         corner[4];

	/** Two edges of the box extended away from corner[0]. */
	Vector2         axis[2];

	/** origin[a] = corner[0].dot(axis[a]); */
	double          minProjLength[2];		// 原点 0点在两个轴上的投影
	double			maxProjLength[2];		// 2点在两个轴上的投影

											/** Returns true if other overlaps one dimension of this. */
	bool Overlaps1Way(const OBB2D& other) const;


	/** Updates the axes after the corners move.  Assumes the
	corners actually form a rectangle. */
	void ComputeAxes();

public:

	OBB2D(const Vector2& center, const double w, const double h, double angle);

	void UpdateAngle(const Vector2& center, const double w, const double h, double angle);

	/** For testing purposes. */
	void MoveTo(const Vector2& center);

	/** Returns true if the intersection of the boxes is non-empty. */
	bool overlaps(const OBB2D& other) const;

	void Render() const;
};
