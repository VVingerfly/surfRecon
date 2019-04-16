#include "marchingCubes.h"
#include "marchingCubesExtended.h"

#include <iostream>
#include <cstdio>
#include <string>
#include <vector>
using namespace std;

void tsdf_writer(string filename, vector<float> &tsdf_data, int dims)
{
	int volume_width = dims;
	int volume_height = dims;
	int volume_depth = dims;
	//FILE *fp = fopen(filename.c_str(), "wb");
	FILE *fp = fopen(filename.c_str(), "w");
	if (fp)
	{
		fwrite(&tsdf_data[0], sizeof(float), volume_width * volume_height * volume_depth, fp);
		fclose(fp);
	}
	printf("write tsdf to %s done!\n", filename.c_str());
}

void tsdf_reader(string filename, vector<float> &tsdf_data, int dims)
{
	int volume_width = dims;
	int volume_height = dims;
	int volume_depth = dims;
	//load volume
	FILE *frp = fopen(filename.c_str(), "rb");
	int num_voxels = volume_width * volume_height * volume_depth;  //256*256*256
	printf("num_voxels is %d, %d, %d\n", volume_width, volume_height, volume_depth);
	//vector<float> tsdf_data;
	tsdf_data.clear();
	tsdf_data.resize(num_voxels);
	if (fread(&tsdf_data[0], sizeof(float), num_voxels, frp) != num_voxels)
	{
		fclose(frp);
		printf("load tsdf failed!\n");
		return;
	}
	fclose(frp);

	printf("read tsdf from %s done!\n", filename.c_str());
}

Grid tsdf_grid(const vector<float> &tsdf_data, int dims)
{
	// setup Marching Cubes grid
	int res = dims;
	Grid  grid(Vector3d(0, 0, 0),
		Vector3d(1, 0, 0),
		Vector3d(0, 1, 0),
		Vector3d(0, 0, 1),
		dims, dims, dims);
	int i = 0;
	for (int x = 0; x < res; ++x)
	for (int y = 0; y < res; ++y)
	for (int z = 0; z < res; ++z)
		grid.value(x, y, z) = tsdf_data.at(i++);
		
	return grid;
}


int main(int argc, char * argv[])
{
	cout << DATAPATH << endl;
	string        filename = DATAPATH "torus";
	unsigned int  res = 256;

	if (argc > 1)
	{
		filename = argv[1];
		res = atoi(argv[2]);
	}
	cout << "file path : " << filename << endl;
	cout << "resolution: " << res << endl;
	vector<float> tsdf_data;
	tsdf_reader(filename + ".tsdf", tsdf_data, res);

	Grid  grid = tsdf_grid(tsdf_data, res);
	std::cout << "Grid generation done." << std::endl;


	// extract 0-level iso-surface
	Mesh   mesh;
	string fileOut;

#define USE_MARCHING_CUBES 1

#if USE_MARCHING_CUBES
	MC mc(grid, mesh);
	fileOut = filename + "_mc.obj";
#else
	MCE mce(grid, mesh, 30);
	fileOut = filename + "_mce.obj";
#endif
	
	std::cout << "Extract surface done." << std::endl;
	write_mesh(mesh, fileOut);
	std::cout << "Write mesh to " << fileOut << " done." << std::endl;
	return 0;
}