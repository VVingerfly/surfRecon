# surfRecon

Surface reconstruction from truncated signed distance function (TSDF).

Implementations of Marching Cubes and Extended Marching Cubes algorithms.
The code is modified and simplified from the original implementaion [IsoEx](https://www.graphics.rwth-aachen.de/IsoEx/).

The code depends on `OpenMesh` and `Eigen`, which are included in the repo already. It was tested under Windows 10 and macOS 10.14.3.

## Usage
### Windows
Use CMake-GUI to generate VS solution, then open it with VS.

### macOS

Open the terminal on the project root directory, then

```bash
mkdir build
cd build
cmake ..
make
```

## Todo
- add Dual Marching Cubes algorithm.

## Reference:  
- Kobbelt, Botsch, Schwanecke, Seidel, [Feature Sensitive Surface Extraction from Volume Data](https://www.graphics.rwth-aachen.de/media/papers/feature1.pdf), Siggraph 2001.
- Lorensen, William E. and Cline, Harvey E, Marching Cubes: A High Resolution 3D Surface Construction Algorithm, Siggraph 1987.
