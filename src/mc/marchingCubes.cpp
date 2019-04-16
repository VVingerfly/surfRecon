#include "marchingCubes.h"
#include "mcTable.h"

#include <iostream>

MC::~MC()
{
}

MC::MC(const Grid& _grid, Mesh& _mesh, float _iso)
	: grid_(_grid),	mesh_(_mesh), iso_(_iso)
{
	CubeIterator cube_it(grid_.begin()), cube_end(grid_.end());
	for (; cube_it != cube_end; ++cube_it)
		process_cube(*cube_it);
}


void MC::process_cube(CubeIdx _cidx)
{
	PointIdx           corner[8];
	VertexHandle       samples[12];
	unsigned char      cubetype(0);
	unsigned int       i;


	// get point indices of corner vertices
	for (i = 0; i < 8; ++i)
		corner[i] = grid_.point_idx(_cidx, i);


	// determine cube type
	for (i = 0; i < 8; ++i)
		if (grid_.scalar_distance(corner[i]) > iso_)
			cubetype |= (1 << i);


	// trivial reject ?
	if (cubetype == 0 || cubetype == 255)
		return;

	// compute samples on cube's edges
	if (edgeTable[cubetype] & 1)    samples[0] = add_vertex(corner[0], corner[1]);
	if (edgeTable[cubetype] & 2)    samples[1] = add_vertex(corner[1], corner[2]);
	if (edgeTable[cubetype] & 4)    samples[2] = add_vertex(corner[3], corner[2]);
	if (edgeTable[cubetype] & 8)    samples[3] = add_vertex(corner[0], corner[3]);
	if (edgeTable[cubetype] & 16)   samples[4] = add_vertex(corner[4], corner[5]);
	if (edgeTable[cubetype] & 32)   samples[5] = add_vertex(corner[5], corner[6]);
	if (edgeTable[cubetype] & 64)   samples[6] = add_vertex(corner[7], corner[6]);
	if (edgeTable[cubetype] & 128)  samples[7] = add_vertex(corner[4], corner[7]);
	if (edgeTable[cubetype] & 256)  samples[8] = add_vertex(corner[0], corner[4]);
	if (edgeTable[cubetype] & 512)  samples[9] = add_vertex(corner[1], corner[5]);
	if (edgeTable[cubetype] & 1024) samples[10] = add_vertex(corner[2], corner[6]);
	if (edgeTable[cubetype] & 2048) samples[11] = add_vertex(corner[3], corner[7]);



	// connect samples by triangles
	for (i = 0; triTable[cubetype][0][i] != -1; i += 3)
		mesh_.add_face(samples[triTable[cubetype][0][i]],
			samples[triTable[cubetype][0][i + 1]],
			samples[triTable[cubetype][0][i + 2]]);
}


//-----------------------------------------------------------------------------


VertexHandle MC::add_vertex(PointIdx _p0, PointIdx _p1)
{
	// find vertex if it has been computed already
	VertexHandle   vh = edge2vertex_.find(_p0, _p1);
	if (vh.is_valid())  return vh;

	// generate new vertex
	Vector3d gp0 = grid_.point(_p0);
	Vector3d gp1 = grid_.point(_p1);
	const Mesh::Point p0 = Mesh::Point(gp0(0), gp0(1), gp0(2));
	const Mesh::Point p1 = Mesh::Point(gp1(0), gp1(1), gp1(2));
	
	float s0 = grid_.scalar_distance(_p0);
	float s1 = grid_.scalar_distance(_p1);
	if (fabs(s1 - s0) > 0.00000001)
	{
		float t = (iso_ - s0) / (s1 - s0);
		vh = mesh_.add_vertex(typename Mesh::Point(p0 + (p1 - p0)*t));
		edge2vertex_.insert(_p0, _p1, vh);
		return vh;
	}
	else
	{
		std::cout << s0 << ", " << s1 << "!!!!!!!!!!!!!!!!!" << (s1 - s0) << std::endl;
		return vh;
	}
}