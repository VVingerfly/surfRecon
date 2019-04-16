#ifndef GRID_H_
#define GRID_H_

#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class Grid
{
public:
	/// CubeIdx can be used to refer to cubes
	typedef unsigned int CubeIdx;
	/// The grid points can be referred to by PointIdx
	typedef unsigned int PointIdx;

	// ray plane intersections
	struct Intersection
	{
		double lambda;
		bool   enter;
	};

	class CubeIterator
	{
	public:
		/** Constructor.
		\internal
		*/
		CubeIterator(unsigned int _idx) : idx_(_idx) {}
		/// Get cube index from iterator
		CubeIdx& operator*() { return idx_; }
		/// Get cube index from iterator
		CubeIdx* operator->() { return &idx_; }
		/// Comparison
		bool operator==(const CubeIterator& _rhs) const { return idx_ == _rhs.idx_; }
		/// Comparison
		bool operator!=(const CubeIterator& _rhs) const { return !(*this == _rhs); }
		/// Pre-increment
		CubeIterator& operator++() { ++idx_; return *this; }
	private:
		unsigned int idx_;
	};
	/// Returns begin iterator for the grid's cubes
	CubeIterator begin() const { return CubeIterator(0); }

	/// Returns end iterator for the grid's cubes
	CubeIterator end()   const { return CubeIterator(n_cubes()); }

public:
	Grid();
	/** Constructor: given the implicit to be sampled, the grids extend
	in 3-space (origin and 3 axes) as well as the resolution (number
	of steps) of the axes. The grid will contain
	_x_res*_y_res*_z_res points and
	(_x_res-1)*(_y_res-1)*(_z_res-1) cubes.
	\note The resolution of each axis has to be less than 1024. This is
	to make sure that a cube or point can be represented by one
	integer.
	*/
	Grid(const Vector3d& _origin = Vector3d(0, 0, 0),
		const Vector3d&  _x_axis = Vector3d(1, 0, 0),
		const Vector3d&  _y_axis = Vector3d(0, 1, 0),
		const Vector3d&  _z_axis = Vector3d(0, 0, 1),
		unsigned int     _x_res = 1,
		unsigned int     _y_res = 1,
		unsigned int     _z_res = 1)
	{
		initialize(_origin, _x_axis, _y_axis, _z_axis, _x_res, _y_res, _z_res);
		values_.resize(this->n_points_, 0.0);
	}
	/// function to initialize the grid
	void initialize(const Vector3d&  _origin,
		const Vector3d&  _x_axis,
		const Vector3d&  _y_axis,
		const Vector3d&  _z_axis,
		unsigned int     _x_res,
		unsigned int     _y_res,
		unsigned int     _z_res);

	~Grid();

	/// Return number of cubes
	unsigned int n_cubes() const { return n_cubes_; }

	/// Return number of points
	unsigned int n_points() const { return n_points_; }

	/// Return the PointIdx of the \b _corners'th corner of the cube \b _idx
	PointIdx point_idx(CubeIdx _idx, unsigned char _corner) const;

	/// Return the 3D point referred to by \b _idx.
	Vector3d  point(PointIdx _idx) const;

	/// Return the 3D point referred to by x,y,z
	Vector3d  point(int x, int y, int z) const;

	/// Return the nearest grid point
	PointIdx nearest_point(const Vector3d &_p);

	const Vector3d& origin() const { return origin_; }
	const Vector3d& x_axis() const { return x_axis_; }
	const Vector3d& y_axis() const { return y_axis_; }
	const Vector3d& z_axis() const { return z_axis_; }
	const Vector3d& dx() const { return dx_; }
	const Vector3d& dy() const { return dy_; }
	const Vector3d& dz() const { return dz_; }

	unsigned int x_resolution() const { return x_res_; }
	unsigned int y_resolution() const { return y_res_; }
	unsigned int z_resolution() const { return z_res_; }

	/// transforms a point to local cube coordinates
	Vector3d to_local(const Vector3d &_pw);

	/// transforms a point for local cube coordinates to world coordinates
	Vector3d to_world(const Vector3d &_pl);

	/// function to intersect a Ray with the cube ( local coordinates )
	bool ray_intersect_local(const Vector3d &_o, const Vector3d &_d, Vector3d  &_entry, Vector3d &_exit);

	bool directed_distance(
		const Vector3d&  _p0, 
		const Vector3d&  _p1,
		Vector3d&        _point,
		Vector3d&        _normal,
		double&          _distance) const;

	///  returns the volume of the grid
	double volume() { return this->x_axis().norm()*this->y_axis().norm()*this->z_axis().norm(); }

	/// returns the outer surface of the grid
	double outer_surface()
	{
		return (this->x_axis().norm()*this->y_axis().norm()*2.0 +
			this->y_axis().norm()*this->z_axis().norm()*2.0 +
			this->x_axis().norm()*this->z_axis().norm()*2.0);
	}

	//////////////////////////////////////////////////////////////////////////
	/// data access
	double& operator()(unsigned int x, unsigned int y, unsigned int z)
	{
		return values_[x + y*this->x_resolution() + z*this->x_resolution()*this->y_resolution()];
	}

	double operator()(unsigned int x, unsigned int y, unsigned int z) const
	{
		return values_[x + y*this->x_resolution() + z*this->x_resolution()*this->y_resolution()];
	}

	double& value(unsigned int x, unsigned int y, unsigned int z)
	{
		return (*this)(x, y, z);
	}

	double value(unsigned int x, unsigned int y, unsigned int z) const
	{
		return (*this)(x, y, z);
	}

	double scalar_distance(PointIdx _pidx) const
	{
		return values_[_pidx];
	}

	bool is_inside(PointIdx _pidx) const
	{
		return values_[_pidx] < 0.0;
	}

	/// get double value, returns 0.0 if position is not in range
	double value_range(int x, int y, int z) const;

	/// function to linearly interpolate a double value at a local point
	double lerp_local(double _x, double _y, double _z);

	/// function to linearly interpolate a double value at a world point
	double lerp_world(double _x, double _y, double _z);

	/// get intersections with iso-surface in local coordinates
	void get_isosurface_intersections_local(const Vector3d &_o,
		const Vector3d &_d,
		double _iso,
		std::vector< Vector3d > &_intersections);

	/// get intersections with iso-surface in world coordinates
	void get_isosurface_intersections_world(const Vector3d &_o,
		const Vector3d &_d,
		double _iso,
		std::vector< Vector3d > &_intersections);

private:
	// matrices which transform points to local and world coordinates
	Matrix4d to_local_;
	Matrix4d to_world_;

	Vector3d   	origin_, x_axis_, y_axis_, z_axis_, dx_, dy_, dz_;
	unsigned int x_res_, y_res_, z_res_, n_cubes_, n_points_;
	CubeIdx      offsets_[8];

	std::vector<double>  values_;
};

#endif  // GRID_H_