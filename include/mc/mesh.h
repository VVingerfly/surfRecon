#ifndef MESH_H_
#define MESH_H_

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

using namespace OpenMesh;
using namespace OpenMesh::IO;

// Define the mesh to be used: need vertex normal and status for EMC
struct MyTraits : public DefaultTraits
{
	VertexAttributes(Attributes::Normal | Attributes::Status);
	HalfedgeAttributes(Attributes::PrevHalfedge);

	/// Use double precision points
	typedef OpenMesh::Vec3d Point;
	/// Use double precision Normals
	typedef OpenMesh::Vec3d Normal;
};
typedef TriMesh_ArrayKernelT<MyTraits>  Mesh;

#endif  // MESH_H_