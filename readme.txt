/////////////////////////////////////////////////////////////////////
// Last Updated 20 April 2012 by Pourya Shirazian
//
// Project: Parsip: PARallel Skeletal Implicit Surfaces Polygonizer 
// Publication:
// Shirazian, P., Wyvill, B., & Duprat, J.-L. (2012). Polygonization 
// of implicit surfaces on Multi-Core Architectures with SIMD 
// instructions. Eurographics Symposium on Parallel Graphics 
// and Visualization. Cagliari, Italy.
/////////////////////////////////////////////////////////////////////

ParsipCmd is the code-base associated with paper titled 
"High performance rendering of implicit surfaces on multi-core architectures with 
SIMD instructions" published at Eurographics 2012 in Cagliari, Italy. 


Implicit Surfaces:

Implicit surfaces are a mathematical representation of 3D Models in graphics. 
They provide:

1. Automatic Blending
2. Integration with constructive solid geometry (CSG) provides union, intersection and
difference operators.
3. Integration with Affine transformations.
4. Proper instancing through affine transformations.
5. Trivial collision Detection for animation (Due to inside/outside property)
6. Compact representation for network-based cooperative modeling

BlobTree:

BlobTree is the title for the scene graph data structure that integrates CSG 
operators, affine tranformations and warping with implicit surfaces primitives.
The scene graph is stored in a text-based script file. The script is parsed and
the graph is restored for rendering.


High performance Rendering:

Polygonization and ray-tracing are two methods for rendering implicit surfaces. 
In this project, a novel algorithm implemented for polygonization or mesh 
extraction from BlobTree scene graphs. Please refer to the paper for detailed 
information about the algorithm.

On multi-core systems with SSE and AVX instruction sets the rendering algorithm is 
scalable to the number of cores available on the system. 

On many-core GPU architectures such as AMD and Nvidia GPUs with many streaming processors
the algorithm has been implemented using OpenCL. CL/GL Interop functionality used for
direct mesh computation and rasterization. 

CPU test machines:
1.Intel Nahalem Architecture with 32 core/64 Threads with SSE Instructions
2.Intel Sandy Bridge Architecture with 6 cores/12 Threads with SSE and AVX
3.Intel Core 2 Due with 2 Cores/2 Threads

GPU test machines:
Nvidia/AMD GPUs listed below are for tested successfully:
1. Nvidia Geforce 8800 Ultra with 512 MB VRAM

List of improvements:

1. Volume visualization with marching cubes method can now benefit from constant time edge table access.
2. Computation of field values on edges with only one SIMD field evaluation 


