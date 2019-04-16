#include "grid.h"
#include <iostream>

Grid::Grid()
{
}


void Grid::initialize(const Vector3d&  _origin,
	const Vector3d&  _x_axis,
	const Vector3d&  _y_axis,
	const Vector3d&  _z_axis,
	unsigned int     _x_res,
	unsigned int     _y_res,
	unsigned int     _z_res)
{
	origin_ = _origin;
	x_axis_ = _x_axis;
	y_axis_ = _y_axis;
	z_axis_ = _z_axis;
	x_res_ = _x_res;
	y_res_ = _y_res;
	z_res_ = _z_res;

	// CubeIdx and PointIdx have 32 bits
	// -> its 3 components should require <= 10 bits
	assert(x_res_ < 1024);
	assert(y_res_ < 1024);
	assert(z_res_ < 1024);

	n_cubes_ = (_x_res - 1) * (_y_res - 1) * (_z_res - 1);
	n_points_ = _x_res * _y_res * _z_res;

	dx_ = x_axis_ / (double)(x_res_ - 1);
	dy_ = y_axis_ / (double)(y_res_ - 1);
	dz_ = z_axis_ / (double)(z_res_ - 1);

	Vector3d x_axisn = x_axis_; x_axisn.normalize();
	Vector3d y_axisn = y_axis_; y_axisn.normalize();
	Vector3d z_axisn = z_axis_; z_axisn.normalize();

	to_world_.setIdentity();
	for (int i = 0; i < 3; ++i)
	{
		to_world_(i, 0) = x_axisn[i];
		to_world_(i, 1) = y_axisn[i];
		to_world_(i, 2) = z_axisn[i];
		to_world_(i, 3) = origin_[i];
	}
	to_local_ = to_world_;
	to_local_.inverse();


	offsets_[0] = 0;
	offsets_[1] = 1;
	offsets_[2] = 1 + x_res_;
	offsets_[3] = x_res_;
	offsets_[4] = x_res_ * y_res_;
	offsets_[5] = 1 + x_res_ * y_res_;
	offsets_[6] = 1 + x_res_ + x_res_ * y_res_;
	offsets_[7] = x_res_ + x_res_ * y_res_;
}


Grid::~Grid()
{
}


Grid::PointIdx Grid::point_idx(CubeIdx _idx, unsigned char _corner) const
{
	assert(_corner < 8);

	// get cube coordinates
	unsigned int X(x_res_ - 1), Y(y_res_ - 1);
	unsigned int x = _idx % X;  _idx /= X;
	unsigned int y = _idx % Y;  _idx /= Y;
	unsigned int z = _idx;

	// transform to point coordinates
	_idx = x + y * x_res_ + z * x_res_ * y_res_;

	// add offset
	return _idx + offsets_[_corner];
}

Vector3d Grid::point(PointIdx _idx) const
{
	unsigned int x = _idx % x_res_;  _idx /= x_res_;
	unsigned int y = _idx % y_res_;  _idx /= y_res_;
	unsigned int z = _idx;

	return origin_ + dx_*x + dy_*y + dz_*z;
}

Vector3d Grid::point(int x, int y, int z) const
{
	return origin_ + dx_*x + dy_*y + dz_*z;
}

Vector3d Grid::to_local(const Vector3d &_pw)
{
	Vector4d pw4;
	pw4 << _pw(0), _pw(1), _pw(2), 1.0;
	Vector4d x = to_local_*pw4;

	if (x(3))
	{
		double w = 1.0 / x(3);
		return Vector3d(x(0)*w, x(1)*w, x(2)*w);
	}
	else return Vector3d(x(0), x(1), x(2));
}

Vector3d Grid::to_world(const Vector3d &_pl)
{
	Vector4d pw4;
	pw4 << _pl(0), _pl(1), _pl(2), 1.0;
	Vector4d x = to_world_*pw4;

	if (x(3))
	{
		double w = 1.0 / x(3);
		return Vector3d(x(0)*w, x(1)*w, x(2)*w);
	}
	else return Vector3d(x(0), x(1), x(2));
}

Grid::PointIdx Grid::nearest_point(const Vector3d &_p)
{
	Vector3d pl = to_local(_p);

	// grid spacing
	double dxf = dx().norm();
	double dyf = dy().norm();
	double dzf = dz().norm();

	// get gird cell
	int x = (int)(pl[0] / dxf);
	int y = (int)(pl[1] / dyf);
	int z = (int)(pl[2] / dzf);

	// transform to point coordinates
	return x + y * x_res_ + z * x_res_ * y_res_;
}

bool Grid::ray_intersect_local(const Vector3d &_o, const Vector3d &_d,
	Vector3d  &_entry, Vector3d &_exit)
{
	// ray cube intersections
	std::vector< Intersection > intersections_;

	// upper,right,front cube corner
	Vector3d cc(x_axis().norm(), y_axis().norm(), z_axis().norm());

	// find ray-cube intersections
	for (unsigned int i = 0; i < 3; ++i)
	{
		if (fabs(_d[i]) > 1E-9)
		{
			// enter
			intersections_.resize(intersections_.size() + 1);
			intersections_.back().lambda = -_o[i] / _d[i];
			if (_d[i] > 0.0)
				intersections_.back().enter = true;
			else
				intersections_.back().enter = false;

			// leaving
			intersections_.resize(intersections_.size() + 1);
			intersections_.back().lambda = (_o[i] - cc[i]) / (-_d[i]);
			if (_d[i] > 0.0)
				intersections_.back().enter = false;
			else
				intersections_.back().enter = true;
		}
	}

	// find last entry and first exit point
	double l_en = -std::numeric_limits<double>::max();
	double l_ex = std::numeric_limits<double>::max();
	for (unsigned int i = 0; i < intersections_.size(); ++i)
	{
		if (intersections_[i].enter)
		{
			if (l_en < intersections_[i].lambda)
				l_en = intersections_[i].lambda;
		}
		else
		{
			if (l_ex > intersections_[i].lambda)
				l_ex = intersections_[i].lambda;
		}
	}

	// does the ray intersect the cube
	if (l_en == -std::numeric_limits<double>::max() || l_ex == std::numeric_limits<double>::max() || l_ex < l_en)
		return false;

	_entry = _o + _d*l_en;
	_exit = _o + _d*l_ex;

	return true;
}

bool Grid::directed_distance(const Vector3d&  _p0,
	const Vector3d&  _p1,
	Vector3d&        _point,
	Vector3d&        _normal,
	double&                  _distance) const
{
	return false;


	Vector3d orig(_p0), dir(_p1 - _p0);
	Vector3d center_;
	double radius_;
	double a = dir.squaredNorm();
	double b = 2.0*(dir.dot(orig - center_));
	double c = (orig - center_).squaredNorm() - radius_*radius_;
	double d = b*b - 4.0*a*c;

	if (d >= 0)
	{
		d = sqrt(d);

		double t1 = (-b - d) / (2.0*a);
		double t2 = (-b + d) / (2.0*a);
		double t = 1.00001;
		if (t1 >= 0.0 && t1 < t) t = t1;
		if (t2 >= 0.0 && t2 < t) t = t2;

		if (t != 1.00001)
		{
			_point = orig + dir*t;
			_normal = (_point - center_) / radius_;
			_distance = ((dir.dot(_normal)) < 0.0) ? dir.norm()*t : -dir.norm()*t;
			return true;
		}
	}

	return false;
}

double Grid::value_range(int x, int y, int z) const
{
	if (x < 0 || x >= (int)this->x_resolution() ||
		y < 0 || y >= (int)this->y_resolution() ||
		z < 0 || z >= (int)this->z_resolution())
		return 0.0;
	else
		return value((unsigned int)x, (unsigned int)y, (unsigned int)z);
}

double Grid::lerp_local(double _x, double _y, double _z)
{
	// grid spacing
	double dxf = this->dx().norm();
	double dyf = this->dy().norm();
	double dzf = this->dz().norm();

	// get gird cell
	int x = (int)(_x / dxf);
	int y = (int)(_y / dyf);
	int z = (int)(_z / dzf);

	// calculate interpolation parameters
	double u = std::max((_x / dxf - double(x)), double(0));
	double v = std::max((_y / dyf - double(y)), double(0));
	double w = std::max((_z / dzf - double(z)), double(0));

	// get values
	double c0 = value_range(x, y, z);
	double c1 = value_range(x + 1, y, z);
	double c2 = value_range(x, y + 1, z);
	double c3 = value_range(x + 1, y + 1, z);

	double c4 = value_range(x, y, z + 1);
	double c5 = value_range(x + 1, y, z + 1);
	double c6 = value_range(x, y + 1, z + 1);
	double c7 = value_range(x + 1, y + 1, z + 1);

	// interpolate
	return   c0 * (1.0 - u) * (1.0 - v) * (1.0 - w)
		+ c1 *     u * (1.0 - v) * (1.0 - w)
		+ c2 * (1.0 - u) * v * (1.0 - w)
		+ c3 * u * v * (1.0 - w) +
		c4 * (1.0 - u) * (1.0 - v) * (w)
		+c5 * u * (1.0 - v) * (w)
		+c6 * (1.0 - u) * v * (w)
		+c7 * u * v * (w);
}

double Grid::lerp_world(double _x, double _y, double _z)
{
	Vector3d pl = to_local(Vector3d(_x, _y, _z));
	return lerp_local(pl[0], pl[1], pl[2]);
}

void Grid::get_isosurface_intersections_world(const Vector3d &_o, const Vector3d &_d, double _iso,
	std::vector< Vector3d > &_intersections)
{
	//Vector3d o = this->to_local_.transform_point(_o);
	//Vector3d d = this->to_local_.transform_vector(_d);

	//get_isosurface_intersections_local(o, d, _iso, _intersections);

	//// transform to world coordinates
	//for (unsigned int i = 0; i < _intersections.size(); ++i)
	//	_intersections[i] = this->to_world_.transform_point(_intersections[i]);
}


void Grid::get_isosurface_intersections_local(const Vector3d &_o, const Vector3d &_d, double _iso,
	std::vector< Vector3d > &_intersections)
{
	_intersections.clear();

	// find cube entry and exit points
	Vector3d entry, exit;
	if (!ray_intersect_local(_o, _d, entry, exit))
	{
		std::cout << "Cube not hit. " << std::endl;
		return;
	}

	// grid spacing
	double dxf = this->dx().norm();
	double dyf = this->dy().norm();
	double dzf = this->dz().norm();

	// get direction offset
	Vector3d ddir = _d;
	ddir.normalize();
	double norm_ddir = std::min(dxf, std::min(dyf, dzf));
	ddir *= norm_ddir;

	// calculate number of steps needed to traverse the volume
	unsigned int n_steps((exit - entry).norm() / norm_ddir);

	double prev_iso(0);
	double cur_iso;

	Vector3d cur_pos = entry;

	// traverse volume
	for (unsigned int i = 0; i < n_steps; ++i)
	{
		// get current double value
		cur_iso = lerp_local(double(cur_pos[0]), double(cur_pos[1]), double(cur_pos[2]));
		if (i == 0) prev_iso = cur_iso;

		// intersect iso-surface?
		if (((prev_iso < _iso) && (_iso < cur_iso)) ||
			((prev_iso > _iso) && (_iso > cur_iso)))
		{
			// refinement of position
			double t = (_iso - prev_iso) / (cur_iso - prev_iso);
			_intersections.push_back(cur_pos + ddir*(t - 1.0));
		}

		prev_iso = cur_iso;
		cur_pos += ddir;
	}
}
