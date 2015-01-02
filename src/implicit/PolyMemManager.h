/*
 * PS_PolyMemManager.h
 *
 *  Created on: 2012-01-16
 *      Author: pourya
 */

#ifndef PS_POLYMEMMANAGER_H_
#define PS_POLYMEMMANAGER_H_

#include "Polygonizer.h"
#include "graphics/SGMesh.h"

using namespace PS::SG;
using namespace PS::GL;

namespace PS{
namespace SIMDPOLY{


//Structure Holding Polygonization Partition Units
class PolyMPUs{
public:
	PolyMPUs()	{ init();}
	~PolyMPUs() {cleanup();}

	void init();
	bool allocate(const vec3i& workDim, const MPUDim& mpuDim);
	void cleanup();

	int setLowerVertex(const vec3f& start, float mpuSide);

	vec3i getWorkDim() const {return m_workDim;}
	U32 countWorkUnits() const {return m_ctWorkUnits;}
private:
	U32 m_ctAllocated;
	U32 m_ctWorkUnits;
	vec3i m_workDim;


public:
	MPU* lpMPUs;
	MPUGLOBALMESH globalMesh;
};


//SimdPoly
class SimdPoly : public SGMesh {
public:
	SimdPoly();
	virtual ~SimdPoly();

	void cleanup();
	void allocate();
	void printModelInfo();
	//void printBBoxInfo();

	//Read Model
	bool readModel(const char* lpFilePath);

	/*!
	 * Create a BlobTree of spheres.
	 * @param ctSpheres Number of spheres to produce.
	 */
	void createRandomSpheresModel(U32 ctSpheres);

	static vec3i CountMPUNeeded(const MPUDim& mpuDim, float cellsize, const vec3f& lo, const vec3f& hi);

	/*!
	 * Prepares bounding boxes of all primitives and operators in the BlobTree.
	 * Compute the union of all primitive bounding boxes as the model BBox.
	 * Counts MPUs needed and allocates them.
	 * @param cellsize the cubic cell size for polygonization
	 * @return success or an error code
	 */
	int prepareBBoxes(float cellsize, U8 mpuDim);

	/*!
	 * Polygonizes a BlobTree model using SIMD optimized polygonizer and records all
	 * related stats.
	 * @param cellsize the cubic cell side length for polygonization
	 * @param bScalarRun running code in scalar mode (with no SIMD optimization)
	 * @param lpProcessStats pointer to an array of MPUSTATS to record poylgonization statistics.
	 * @return success or an error code
	 */
	int polygonize(float cellsize, bool bScalarRun,
				   MPUSTATS* lpProcessStats);

	//Produces Single Mesh vertex and element buffer objects
	bool extractSingleMeshObject();

	//Draw Mesh Normals for Debug
	void drawMeshNormals();


	//Access
	PolyMPUs* getMPUs() {return &m_polyMPUs;}
	SOABlobPrims* getBlobPrims() const {return m_lpBlobPrims;}
	SOABlobOps* getBlobOps() const {return m_lpBlobOps;}
	SOABlobNodeMatrices* getMtxNode() const {return m_lpMtxNode;}
	SOABlobBoxMatrices* getMtxBox() const {return m_lpMtxBox;}

private:

	int prepareOpBBox(int idxOp, vec3f& boxLo, vec3f& boxHi);

private:
	SOABlobPrims* m_lpBlobPrims;
	SOABlobOps* m_lpBlobOps;
	SOABlobNodeMatrices* m_lpMtxNode;
	SOABlobBoxMatrices* m_lpMtxBox;
	PolyMPUs m_polyMPUs;
	MPUDim m_mpuDim;

	void* m_lpMemBlock;
	U32 m_szInputData;
};



}
}

#endif /* PS_POLYMEMMANAGER_H_ */
