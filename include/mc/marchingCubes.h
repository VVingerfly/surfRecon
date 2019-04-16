#ifndef MARCHING_CUBES_H_
#define MARCHING_CUBES_H_

#include "grid.h"
#include "mesh.h"
#include "edge2vertexMap.h"

class MC
{
public:
	MC(const Grid& _grid, Mesh& _mesh, float _iso = 0.0);
	~MC();

private:

	typedef typename Grid::PointIdx      PointIdx;
	typedef typename Grid::CubeIdx       CubeIdx;
	typedef typename Grid::CubeIterator  CubeIterator;

	typedef typename Mesh::Point         Point;
	typedef typename Mesh::VertexHandle  VertexHandle;
	typedef typename Mesh::FaceHandle    FaceHandle;



	void process_cube(CubeIdx _idx);
	VertexHandle add_vertex(PointIdx _p0, PointIdx _p1);


	const Grid&      grid_;
	Mesh&            mesh_;
	float            iso_;

	// maps an edge to the sample vertex generated on it
	Edge2VertexMapT<PointIdx, VertexHandle> edge2vertex_;
};

#endif  // MARCHING_CUBES_H_