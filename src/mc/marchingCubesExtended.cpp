#include "marchingCubesExtended.h"
#include "mcTable.h"

#include <iostream>

MCE::MCE(const Grid& _grid, Mesh& _mesh, float _feature_angle)
	: grid_(_grid),	mesh_(_mesh), feature_angle_(_feature_angle / 180.0f * M_PI), n_edges_(0), n_corners_(0)
{
	CubeIterator cube_it(grid_.begin()), cube_end(grid_.end());
	for (; cube_it != cube_end; ++cube_it)
		process_cube(*cube_it);

	flip_edges();

	std::cerr << "Found "
		<< n_edges_ << " edge features, "
		<< n_corners_ << " corner features\n";
}


void MCE::process_cube(CubeIdx _idx)
{
	PointIdx           corner[8];
	VertexHandle       samples[12];
	unsigned char      cubetype(0);
	unsigned int       i, j;
	unsigned int       n_components, n_vertices;
	int                *indices;
	VertexHandle       vh;
	std::vector<VertexHandle> vhandles;



	// get corner vertices
	for (i = 0; i < 8; ++i)
		corner[i] = grid_.point_idx(_idx, i);


	// determine cube type
	for (i = 0; i < 8; ++i)
		if (!grid_.is_inside(corner[i]))
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
	n_components = triTable[cubetype][1][0];
	indices = &triTable[cubetype][1][n_components + 1];

	for (i = 1; i <= n_components; ++i)  // sheets in this voxel
	{
		// current sheet contains n_vertices vertices
		n_vertices = triTable[cubetype][1][i];


		// collect vertices of n-gon
		vhandles.clear();
		for (j = 0; j < n_vertices; ++j)
			vhandles.push_back(samples[indices[j]]);


		// look for a feature
		vh = find_feature(vhandles);


		// feature -> create triangle fan around feature vertex
		if (vh.is_valid())
		{
			vhandles.push_back(vhandles[0]);
			for (j = 0; j < n_vertices; ++j)
				mesh_.add_face(vhandles[j], vhandles[j + 1], vh);
		}


		// no feature -> old marching cubes triangle table
		else
		{
			for (j = 0; polyTable[n_vertices][j] != -1; j += 3)
				mesh_.add_face(samples[indices[polyTable[n_vertices][j]]],
					samples[indices[polyTable[n_vertices][j + 1]]],
					samples[indices[polyTable[n_vertices][j + 2]]]);
		}

		indices += n_vertices;
	}
}


VertexHandle MCE::add_vertex(PointIdx _p0, PointIdx _p1)
{
	// find vertex if it has been computed already
	VertexHandle   vh = edge2vertex_.find(_p0, _p1);
	if (vh.is_valid())  return vh;



	// generate new vertex
	Vector3d gp0 = grid_.point(_p0);
	Vector3d gp1 = grid_.point(_p1);
	Vector3d gpoint, gnormal(0.0, 0.0, 0.0);
	double distance;
	bool ok = grid_.directed_distance(gp0, gp1, gpoint, gnormal, distance);

	const Mesh::Point p0 = Mesh::Point(gp0(0), gp0(1), gp0(2));
	const Mesh::Point p1 = Mesh::Point(gp1(0), gp1(1), gp1(2));
	Mesh::Point point  = Mesh::Point(gpoint(0), gpoint(1), gpoint(2));
	Mesh::Point normal = Mesh::Point(gnormal(0), gnormal(1), gnormal(2));
	if (!ok)
	{
		// should not happen, just in case of precision errors...
		float s0 = fabs(grid_.scalar_distance(_p0));
		float s1 = fabs(grid_.scalar_distance(_p1));
		float t = s0 / (s0 + s1);
		point = (1.0f - t)*p0 + t*p1;
	}


	// add vertex
	vh = mesh_.add_vertex(point);
	mesh_.set_normal(vh, normal);
	edge2vertex_.insert(_p0, _p1, vh);


	return vh;
}


VertexHandle MCE::find_feature(const VertexHandleVector& _vhandles)
{
	size_t i, j, nV = _vhandles.size(), rank;

	// collect point & normals;
	std::vector<typename Mesh::Point>  p, n;
	p.resize(nV);
	n.resize(nV);
	for (i = 0; i < nV; ++i)
	{
		p[i] = mesh_.point(_vhandles[i]);
		n[i] = mesh_.normal(_vhandles[i]);
	}

	// move barycenter of points into
	typename Mesh::Point cog(0, 0, 0);
	for (i = 0; i < nV; ++i)  cog += p[i];
	cog /= (float)nV;
	for (i = 0; i < nV; ++i)  p[i] -= cog;

	// normal angle criterion
	double  c, min_c, max_c;
	typename Mesh::Point  axis;
	for (min_c = 1.0, i = 0; i < nV; ++i)
		for (j = 0; j < nV; ++j)
			if ((c = (n[i] | n[j])) < min_c)
			{
				min_c = c;
				axis = n[i] % n[j];
			}


	// angle to small, no feature -> return invalid vertex handle
	if (min_c > cos(feature_angle_))
		return Mesh::InvalidVertexHandle;



	// ok, we have a feature
	// is it edge or corner, i.e. rank 2 or 3 ?
	axis.normalize();
	for (min_c = 1.0, max_c = -1.0, i = 0; i < nV; ++i)
	{
		c = (axis | n[i]);
		if (c < min_c)  min_c = c;
		if (c > max_c)  max_c = c;
	}
	c = std::max(fabs(min_c), fabs(max_c));
	c = sqrt(1.0 - c*c);
	rank = (c > cos(feature_angle_) ? 2 : 3);

	if (rank == 2)  ++n_edges_;
	else            ++n_corners_;


	// setup linear system (find intersection of tangent planes)
	MatrixXd A(nV, 3);
	VectorXd b(nV);

	for (i = 0; i < nV; ++i) {
		A(i, 0) = n[i][0];
		A(i, 1) = n[i][1];
		A(i, 2) = n[i][2];
		b(i) = (p[i] | n[i]);
	}



	//// SVD of matrix A
	//Matrix3d V;
	//VectorXd S(nV);
	//Math::MatrixT<double>  V(3, 3);
	//Math::VectorT<double>  S(nV);
	//Math::svd_decomp(A, S, V);
	//JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

	//cout <<  endl << svd.matrixU() << endl;
	//cout << << endl << svd.matrixV() << endl;

	//// rank == 2 -> suppress smallest singular value
	//if (rank == 2) {
	//	double smin = FLT_MAX;
	//	unsigned int sminid = 0;
	//	unsigned int srank = std::min(nV, 3u);

	//	for (i = 0; i < srank; ++i)
	//		if (S(i) < smin) {
	//			smin = S(i);
	//			sminid = i;
	//		}

	//	S(sminid) = 0.0;
	//}

	//// SVD backsubstitution -> least squares, least norm solution x
	//Math::VectorT<double>  x(3);
	//Math::svd_backsub(A, S, V, b, x);

	JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
	Vector3d x = svd.solve(b);


	// transform x to world coords
	typename Mesh::Point point(x(0), x(1), x(2));
	point += cog;



	// insert the feature-point 
	VertexHandle vh = mesh_.add_vertex(point);
	mesh_.status(vh).set_feature(true);


	return vh;
}



void MCE::flip_edges()
{
	VertexHandle v0, v1, v2, v3;
	typename Mesh::HalfedgeHandle he;

	typename Mesh::EdgeIter e_it(mesh_.edges_begin()), e_end(mesh_.edges_end());

	for (; e_it != e_end; ++e_it) {
		if (mesh_.is_flip_ok(*e_it)) {
			he = mesh_.halfedge_handle(*e_it, 0);
			v0 = mesh_.to_vertex_handle(he);
			he = mesh_.next_halfedge_handle(he);
			v1 = mesh_.to_vertex_handle(he);
			he = mesh_.halfedge_handle(*e_it, 1);
			v2 = mesh_.to_vertex_handle(he);
			he = mesh_.next_halfedge_handle(he);
			v3 = mesh_.to_vertex_handle(he);

			// flip edge if it would connect two features (v1, v3)
			// and not disconnect two others (v0, v2) afterwards
			// (maybe we should check for flipping triangle normals)
			if (mesh_.status(v1).feature() && mesh_.status(v3).feature() && !mesh_.status(v0).feature() && !mesh_.status(v2).feature())
				mesh_.flip(*e_it);
		}
	}
}
