#ifndef MARCHING_CUBES_EXTENDED_H_
#define MARCHING_CUBES_EXTENDED_H_

#include "grid.h"
#include "mesh.h"
#include "edge2vertexMap.h"

class MCE
{
public:
	MCE(const Grid&  _grid, Mesh&        _mesh, float       _feature_angle);

private:
	typedef typename Grid::PointIdx        PointIdx;
	typedef typename Grid::CubeIdx         CubeIdx;
	typedef typename Grid::CubeIterator    CubeIterator;
	typedef typename Mesh::VertexHandle    VertexHandle;
	typedef std::vector<VertexHandle>      VertexHandleVector;


	void process_cube(CubeIdx _idx);

	VertexHandle add_vertex(PointIdx _p0, PointIdx _p1);
	VertexHandle find_feature(const VertexHandleVector& _vhandles);

	void flip_edges();

	const Grid&      grid_;
	Mesh&            mesh_;

	float            feature_angle_;
	unsigned int     n_edges_, n_corners_;

	// maps an edge to the sample vertex generated on it
	Edge2VertexMapT<PointIdx, VertexHandle> edge2vertex_;
};

#endif  // MARCHING_CUBES_EXTENDED_H_