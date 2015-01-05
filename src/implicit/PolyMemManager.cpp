/*
 * PS_PolyMemManager.cpp
 *
 *  Created on: 2012-01-16
 *      Author: pourya
 */
#include "PolyMemManager.h"
#include "ReadSceneModel.h"

#include <vector>
#include <GL/glew.h>
#include <tbb/cache_aligned_allocator.h>

using namespace tbb;

namespace PS{
namespace SIMDPOLY{

U64 MakeSIMDPadSize(U64 sz) {
	return (((U64)(sz) + (PS_SIMD_FLEN-1)) & ~(PS_SIMD_FLEN-1));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void PolyMPUs::init()
{
	lpMPUs = NULL;
	m_ctAllocated = 0;
	m_workDim 	  = vec3i(0, 0, 0);
	m_ctWorkUnits = 0;
}

bool PolyMPUs::allocate(const vec3i& workDim, const MPUDim& mpuDim)
{
	U32 ctNeeded = workDim.x * workDim.y * workDim.z;
	this->m_workDim = workDim;
	this->m_ctWorkUnits = ctNeeded;
	if((lpMPUs != NULL)&&(m_ctAllocated >= ctNeeded))
	{
		printf("Already have enough memory for MPUs. Needed=%u Allocated=%u\n", m_ctWorkUnits, m_ctAllocated);
		return false;
	}
	cleanup();
	printf("Allocating memory for MPUs. Dim=[%d %d %d], Needed=%u \n", m_workDim.x, m_workDim.y, m_workDim.z, m_ctWorkUnits);

	lpMPUs = AllocAligned<MPU>(ctNeeded);



	globalMesh.mpuMeshVertexStrideF = MakeSIMDPadSize(mpuDim.countCells() * MAX_VERTICES_OUTPUT_PER_CELL * 3);
	globalMesh.mpuMeshTriangleStrideU32 = MakeSIMDPadSize(mpuDim.countCells() * MAX_TRIANGLES_OUTPUT_PER_CELL * 3);

	globalMesh.globalMeshVertexBufferSizeF = PS_SIMD_PADSIZE(globalMesh.mpuMeshVertexStrideF * ctNeeded);
	globalMesh.globalMeshTriangleBufferSizeU32 = PS_SIMD_PADSIZE(globalMesh.mpuMeshTriangleStrideU32 * ctNeeded);

	globalMesh.vColor = tbb::cache_aligned_allocator<float>().allocate(globalMesh.globalMeshVertexBufferSizeF);
	globalMesh.vNorm  = tbb::cache_aligned_allocator<float>().allocate(globalMesh.globalMeshVertexBufferSizeF);
	globalMesh.vPos   = tbb::cache_aligned_allocator<float>().allocate(globalMesh.globalMeshVertexBufferSizeF);
	globalMesh.vTriangles = tbb::cache_aligned_allocator<U32>().allocate(globalMesh.globalMeshTriangleBufferSizeU32);

	//Total Memory size
	globalMesh.memSizeInBytes = globalMesh.globalMeshVertexBufferSizeF * 3 * sizeof(float) +
								globalMesh.globalMeshTriangleBufferSizeU32 * sizeof(U32) +
								sizeof(MPUGLOBALMESH);

	m_ctAllocated = ctNeeded;
	return true;
}

int PolyMPUs::setLowerVertex(const vec3f& start, float mpuSide)
{
	if((m_ctAllocated == 0)||(m_ctWorkUnits == 0))
		return 0;

	U32 idxMPU = 0;
	//Create all MPUs
	//Create all intersecting MPUs
	for (int i = 0; i < m_workDim.x; i++) {
		for (int j = 0; j < m_workDim.y; j++) {
			for (int k = 0; k < m_workDim.z; k++)
			{
				lpMPUs[idxMPU].bboxLo = vec3f::add(start, vec3f((float) i * mpuSide, (float) j * mpuSide, (float) k * mpuSide));
				lpMPUs[idxMPU].idxGlobalID = idxMPU;
				idxMPU++;
			}
		}
	}

	return idxMPU;
}

void PolyMPUs::cleanup()
{
	printf("Free memory for MPUs.\n");
	FreeAligned(lpMPUs);
	lpMPUs = 0;

	if(globalMesh.vPos)
		tbb::cache_aligned_allocator<float>().deallocate(globalMesh.vPos, globalMesh.globalMeshVertexBufferSizeF);
	if(globalMesh.vColor)
		tbb::cache_aligned_allocator<float>().deallocate(globalMesh.vColor, globalMesh.globalMeshVertexBufferSizeF);
	if(globalMesh.vNorm)
		tbb::cache_aligned_allocator<float>().deallocate(globalMesh.vNorm, globalMesh.globalMeshVertexBufferSizeF);
	if(globalMesh.vTriangles)
		tbb::cache_aligned_allocator<U32>().deallocate(globalMesh.vTriangles, globalMesh.globalMeshTriangleBufferSizeU32);


	globalMesh.vPos = NULL;
	globalMesh.vColor = NULL;
	globalMesh.vNorm = NULL;
	globalMesh.vTriangles = NULL;

	m_ctAllocated = 0;
}



//////////////////////////////////////////////////////////////////////////////////////////////
SimdPoly::SimdPoly(): SGMesh()
{
	m_lpBlobOps = NULL;
	m_lpBlobPrims = NULL;
	m_lpMtxBox = NULL;
	m_lpMtxNode = NULL;

	m_lpMemBlock = NULL;
	m_szInputData = 0;
}

SimdPoly::~SimdPoly() {
	cleanup();
}

void SimdPoly::cleanup() {
	FreeAligned(m_lpMemBlock);
	m_lpMemBlock = NULL;
	m_polyMPUs.cleanup();
}

bool SimdPoly::readModel(const char* lpFilePath)
{
	ModelReader* reader = new ModelReader(*this);
	int res = reader->read(lpFilePath);
	SAFE_DELETE(reader);

	return (res == MODELREAD_SUCCESS);
}

void SimdPoly::createRandomSpheresModel(U32 ctSpheres)
{
	vec3f animBoxLo = vec3f(-2.0f, -2.0f, -2.0f);
	vec3f animBoxHi = vec3f(2.0f, 2.0f, 2.0f);
	vec3f pos;
	vec3f color;

	//Should be less than PADSIZE
	m_lpBlobPrims->count = ctSpheres;
	for(U32 i=0; i<m_lpBlobPrims->count; i++)
	{
		pos.x = RandRangeT<float>(animBoxLo.x, animBoxHi.x);
		pos.y = RandRangeT<float>(animBoxLo.y, animBoxHi.y);
		pos.z = RandRangeT<float>(animBoxLo.z, animBoxHi.z);

		color = pos;
		color.normalize();

		m_lpBlobPrims->posX[i] = pos.x;
		m_lpBlobPrims->posY[i] = pos.y;
		m_lpBlobPrims->posZ[i] = pos.z;

		m_lpBlobPrims->colorX[i] = color.x;
		m_lpBlobPrims->colorY[i] = color.y;
		m_lpBlobPrims->colorZ[i] = color.z;
	}
}

void SimdPoly::printModelInfo()
{
	if(m_lpMemBlock == NULL) return;
	printf("Prims# %d, Ops# %d, Prim Matrices# %d, Box Matrices# %d.\n",
	m_lpBlobPrims->count, m_lpBlobOps->count, m_lpMtxNode->count, m_lpMtxBox->count);
}

void SimdPoly::allocate()
{
	printf("Allocating memory for input Blobtree.\n");
	m_szInputData = sizeof(SOABlobPrims) + sizeof(SOABlobOps) + sizeof(SOABlobNodeMatrices) + sizeof(SOABlobBoxMatrices) + PS_SIMD_ALIGN_SIZE * 4;
	m_lpMemBlock = AllocAligned(m_szInputData);

	m_lpBlobPrims 		= reinterpret_cast<SOABlobPrims*>(m_lpMemBlock);
	m_lpBlobOps 		= reinterpret_cast<SOABlobOps*>((U64)m_lpMemBlock + sizeof(SOABlobPrims));
	m_lpMtxNode 	= reinterpret_cast<SOABlobNodeMatrices*>((U64)m_lpBlobOps + sizeof(SOABlobOps));
	m_lpMtxBox = reinterpret_cast<SOABlobBoxMatrices*>((U64)m_lpMtxNode + sizeof(SOABlobNodeMatrices));
}


//Returns the count of needed MPUs
vec3i SimdPoly::CountMPUNeeded(const MPUDim& mpuDim, float cellsize, const vec3f& lo, const vec3f& hi)
{
	const int cellsPerMPU = mpuDim.dim() - 1;

	vec3f allSides = vec3f::sub(hi, lo);
	vec3i ctCellsNeeded;
	vec3i ctMPUNeeded;

	ctCellsNeeded.x = static_cast<int>(ceil(allSides.x / cellsize));
	ctCellsNeeded.y = static_cast<int>(ceil(allSides.y / cellsize));
	ctCellsNeeded.z = static_cast<int>(ceil(allSides.z / cellsize));

	ctMPUNeeded.x = ctCellsNeeded.x / cellsPerMPU;
	ctMPUNeeded.y = ctCellsNeeded.y / cellsPerMPU;
	ctMPUNeeded.z = ctCellsNeeded.z / cellsPerMPU;
	if (ctCellsNeeded.x % cellsPerMPU != 0)
		ctMPUNeeded.x++;
	if (ctCellsNeeded.y % cellsPerMPU != 0)
		ctMPUNeeded.y++;
	if (ctCellsNeeded.z % cellsPerMPU != 0)
		ctMPUNeeded.z++;

	return ctMPUNeeded;
}

//Bounding Boxes computation and MPU Allocation
int SimdPoly::prepareBBoxes(float cellsize, U8 mpuDim)
{
	//setup dimensions
	m_mpuDim.set(mpuDim);

	vec3f opBoxLo, opBoxHi;
	int res = PrepareAllBoxes(*m_lpBlobOps, *m_lpBlobPrims, *m_lpMtxBox);

	//Prepare MPUs Needed for processing
	vec3i workGridDim = CountMPUNeeded(m_mpuDim, cellsize, m_lpBlobPrims->bboxLo, m_lpBlobPrims->bboxHi);
	m_polyMPUs.allocate(workGridDim, mpuDim);
	m_polyMPUs.setLowerVertex(m_lpBlobPrims->bboxLo, (m_mpuDim.dim() - 1)* cellsize);

	return res;
}




//Polygonize
int SimdPoly::polygonize(float cellsize, bool bScalarRun, MPUSTATS* lpProcessStats)
{
	return Polygonize(cellsize,
					  bScalarRun,
					  *m_lpBlobOps,
					  *m_lpBlobPrims,
					  *m_lpMtxNode,
					  m_polyMPUs.countWorkUnits(),
					  m_polyMPUs.lpMPUs,
					  m_mpuDim,
					  &m_polyMPUs.globalMesh,
					  lpProcessStats);
}

//Extract a single mesh from all those MPUs
bool SimdPoly::extractSingleMeshObject()
{
	U32 ctMPUs = m_polyMPUs.countWorkUnits();
	if(ctMPUs == 0)
		return false;

	vector<U32> arrVertexCount;
	arrVertexCount.resize(ctMPUs);

	vector<U32> arrVertexCountScanned;
	arrVertexCountScanned.resize(ctMPUs + 1);

	vector<U32> arrFaceCount;
	arrFaceCount.resize(ctMPUs);

	vector<U32> arrFaceCountScanned;
	arrFaceCountScanned.resize(ctMPUs + 1);


	//Scan Arrays of Vertices Count and Faces Count
	for(U32 i=0; i < ctMPUs; i++)
	{
		arrVertexCount[i] = m_polyMPUs.lpMPUs[i].ctVertices;
		arrFaceCount[i] = m_polyMPUs.lpMPUs[i].ctTriangles;
	}

	//Prefix Sum Scan
	arrVertexCountScanned[0] = 0;
	arrFaceCountScanned[0] = 0;
	for(U32 i=1; i <= ctMPUs; i++)
	{
		arrVertexCountScanned[i] = arrVertexCountScanned[i - 1] + arrVertexCount[i - 1];
		arrFaceCountScanned[i] = arrFaceCountScanned[i - 1] + arrFaceCount[i - 1];
	}

	U32 ctTriangles = arrFaceCountScanned[ctMPUs];
	U32 ctVertices = arrVertexCountScanned[ctMPUs];

	//Mesh components
	vector<float> arrVertices;
	vector<float> arrNormals;
	vector<float> arrColors;
	arrVertices.resize(ctVertices * 3);
	arrNormals.resize(ctVertices * 3);
	arrColors.resize(ctVertices * 3);

	vector<U32> arrFaces;
	arrFaces.resize(ctTriangles * 3);

	const U32 mpuMeshVertexStrideF = m_polyMPUs.globalMesh.mpuMeshVertexStrideF;
	const U32 mpuMeshTriangleStrideU32 = m_polyMPUs.globalMesh.mpuMeshTriangleStrideU32;

	for(U32 i=0; i < ctMPUs; i++)
	{
		if(arrVertexCount[i] > 0)
		{
			memcpy(&arrVertices[arrVertexCountScanned[i] * 3], &m_polyMPUs.globalMesh.vPos[i * mpuMeshVertexStrideF], sizeof(float) * 3 * arrVertexCount[i]);
			memcpy(&arrColors[arrVertexCountScanned[i] * 3], &m_polyMPUs.globalMesh.vColor[i * mpuMeshVertexStrideF], sizeof(float) * 3 * arrVertexCount[i]);
			memcpy(&arrNormals[arrVertexCountScanned[i] * 3], &m_polyMPUs.globalMesh.vNorm[i * mpuMeshVertexStrideF], sizeof(float) * 3 * arrVertexCount[i]);

			U32 offsetFace = arrFaceCountScanned[i] * 3;
			U32 offsetVertex = arrVertexCountScanned[i];
			U32 jMax = arrFaceCount[i]*3;
			for(U32 j=0; j<jMax; j++)
			{
				arrFaces[offsetFace + j] = m_polyMPUs.globalMesh.vTriangles[i * mpuMeshTriangleStrideU32 + j] + offsetVertex;
			}
		}
	}

	//Buffers
	SGMesh::cleanup();
	setupVertexAttribs(arrVertices, 3, gbtPosition);
	setupVertexAttribs(arrColors, 3, gbtColor);
	setupVertexAttribs(arrNormals, 3, gbtNormal);
	setupFaceIndexBuffer(arrFaces, GLFaceType::ftTriangles);


	return 1;
}

void SimdPoly::drawMeshNormals()
{
	if(!GLMeshBuffer::isBufferValid(gbtPosition)) return;

	U32 ctMPUs = m_polyMPUs.countWorkUnits();

	U32 mpuMeshVertexStrideF = m_polyMPUs.globalMesh.mpuMeshVertexStrideF;
	//U32 mpuMeshTriangleStrideU32 = m_polyMPUs.globalMesh.mpuMeshTriangleStrideU32;

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glColor3f(0.0f, 0.0f, 1.0f);
	glBegin(GL_LINES);
	for(U32 i=0; i<ctMPUs; i++)
	{
		U32 ctVertices = m_polyMPUs.lpMPUs[i].ctVertices;
		if(ctVertices > 0)
		{
			for(U32 j=0; j<ctVertices; j++)
			{
				vec3f s, n, e;
				s.load(&m_polyMPUs.globalMesh.vPos[i*mpuMeshVertexStrideF + j*3]);
				n.load(&m_polyMPUs.globalMesh.vNorm[i*mpuMeshVertexStrideF + j*3]);
				e = vec3f::add(s, vec3f::mul(0.3f, n));

				glVertex3f(s.x, s.y, s.z);
				glVertex3f(e.x, e.y, e.z);
			}
		}

	}

	glEnd();
	glPopAttrib();
}





























}
}

