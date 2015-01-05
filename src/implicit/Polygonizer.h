#ifndef POLYGONIZER_H
#define POLYGONIZER_H

#include "tbb/blocked_range.h"
#include "tbb/parallel_for.h"
#include "tbb/enumerable_thread_specific.h"
#include "tbb/task_scheduler_observer.h"
#include "tbb/tick_count.h"
#include "base/SIMDVecN.h"
#include "base/Vec.h"

using namespace PS::MATH;

namespace PS{
namespace SIMDPOLY{

using namespace tbb;
using namespace PS::MATHSIMD;

#define DEFAULT_MPU_DIM 8

//Return codes
#define RET_PARAM_ERROR -1
#define RET_NOT_ENOUGH_MEM -2
#define RET_INVALID_BVH -3
#define RET_SUCCESS 1

#define EDGETABLE_DEPTH 8

//FieldValue Definitions
#define ISO_VALUE 0.5f
#define ISO_DIST  0.45420206f
#define FIELDVALUE_EPSILON 0.001f
#define NORMAL_DELTA 0.0001f
#define DEFAULT_ROOTFINDER_ITERATIONS 5
#define ROOT_THREE 			 1.732050808f
#define CUBEONE_HALFDIAGONAL 0.866f

//CellSize
#define MIN_CELL_SIZE	0.01f
#define DEFAULT_CELL_SIZE 0.14f

//BlobTree
#define MAX_TREE_NODES   1024
#define MAX_MTX_NODES MAX_TREE_NODES * 2
#define PRIM_MATRIX_STRIDE 12
#define BOX_MATRIX_STRIDE 16

#define MAX_BVH_DEPTH_TO_MPU 6

//Strides
#define MAX_THREADS_COUNT 128

//Stats
#define MAX_TRIANGLES_OUTPUT_PER_CELL 5
#define MAX_VERTICES_OUTPUT_PER_CELL 15
//////////////////////////////////////////////////////////
enum CUBEFACES {cfLeft, cfRight, cfBottom, cfTop, cfNear, cfFar};

#ifndef BLOBTREE_DEFINITIONS
enum PrimitiveType {primPoint, primLine, primCylinder, primDisc, primRing, primCube, primTriangle, primQuadricPoint, primNULL, primInstance};
enum OperatorType {opUnion, opIntersect, opDif, opSmoothDif, opBlend, opRicciBlend, opGradientBlend, opFastQuadricPointSet,
				   opCache, opWarpTwist, opWarpTaper, opWarpBend, opWarpShear};
#endif

//MPU Dimension
class MPUDim {
public:
	MPUDim(U8 dim_ = 8) {
		set(dim_);
	}

	MPUDim(const MPUDim& dim) {
		set(dim.dim());
	}

	static bool isPowerOfTwo(U8 x) {
		return (!(x == 0) && !(x & (x - 1)));
	}

	static bool isValid(U8 dim_) {
		return (dim_ % 2 == 0);
	}

	void set(U8 dim_) {
		assert(isValid(dim_));

		m_dimension = dim_;
		m_cellid_shift_x = 0;
		m_cellid_shift_y = log2(dim_);
		m_cellid_shift_z = m_cellid_shift_y * 2;
		m_cellid_bitmask = dim_ - 1;
	}

	U64 hashsize() const {
		return (U64)(1 << (3 * m_cellid_shift_y));
	}

	U64 hash(U32 x, U32 y, U32 z) const {
		return ((((z) & m_cellid_bitmask) << m_cellid_shift_z) | (((y) & m_cellid_bitmask) << m_cellid_shift_y) | ((x) & m_cellid_bitmask));
	}

	U8 dim() const {return m_dimension;}
	U64 dim3() const { return m_dimension * m_dimension * m_dimension;}
	U64 countCells() const {
		U8 d = m_dimension - 1;
		return d * d * d;
	}

	U8 shiftX() const {return m_cellid_shift_x;}
	U8 shiftY() const {return m_cellid_shift_y;}
	U8 shiftZ() const {return m_cellid_shift_z;}
	U8 mask() const {return m_cellid_bitmask;}
private:
	U8 m_dimension;
	U8 m_cellid_shift_x;
	U8 m_cellid_shift_y;
	U8 m_cellid_shift_z;
	U8 m_cellid_bitmask;

};

//Total Data Structure Per Each Core : Including Input BlobPrims, Ops, MPU mesh
//Aligned SOA structure for primitives
//SS = 18 * 128 * 4 + 2 * 128 * 1 + 7 * 4 = 9500 Bytes
struct PS_BEGIN_ALIGNED(PS_SIMD_FLEN) SOABlobPrims
{
	//Primitive Type
	U8 type[PS_SIMD_PADSIZE(MAX_TREE_NODES)];

	//Matrix index
	U16 idxMatrix[PS_SIMD_PADSIZE(MAX_TREE_NODES)];

	//BlobTree primitive attributes
	float posX[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float posY[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float posZ[PS_SIMD_PADSIZE(MAX_TREE_NODES)];

	float dirX[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float dirY[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float dirZ[PS_SIMD_PADSIZE(MAX_TREE_NODES)];

	float resX[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float resY[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float resZ[PS_SIMD_PADSIZE(MAX_TREE_NODES)];

	float colorX[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float colorY[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float colorZ[PS_SIMD_PADSIZE(MAX_TREE_NODES)];

	//Seed Points for Continuations Method
	float seedX[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float seedY[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float seedZ[PS_SIMD_PADSIZE(MAX_TREE_NODES)];

	//Prim
	float bboxLoX[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float bboxLoY[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float bboxLoZ[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float bboxHiX[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float bboxHiY[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float bboxHiZ[PS_SIMD_PADSIZE(MAX_TREE_NODES)];

	vec3f bboxLo;
	vec3f bboxHi;
	U32 count;
} PS_END_ALIGNED(PS_SIMD_FLEN);


//Aligned SOA structure for operators
//SS= 5636
enum OPFLAGS {ofRightChildIsOp = 1, ofLeftChildIsOp = 2, ofChildIndexIsRange = 4, ofIsUnaryOp = 8,  ofIsRightOp = 16, ofBreak = 32};
struct PS_BEGIN_ALIGNED(PS_SIMD_FLEN) SOABlobOps
{
	U8 type[PS_SIMD_PADSIZE(MAX_TREE_NODES)];

	//bit 1: Right Child is op
	//bit 2: Left Child is Op
	//bit 3: Child index is range
	//bit 4: Is Unary Op
	U8 opFlags[PS_SIMD_PADSIZE(MAX_TREE_NODES)];

	U16 opLeftChild[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
   	U16 opRightChild[PS_SIMD_PADSIZE(MAX_TREE_NODES)];

	float bboxLoX[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float bboxLoY[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float bboxLoZ[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float bboxHiX[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float bboxHiY[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float bboxHiZ[PS_SIMD_PADSIZE(MAX_TREE_NODES)];

	float resX[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float resY[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float resZ[PS_SIMD_PADSIZE(MAX_TREE_NODES)];
	float resW[PS_SIMD_PADSIZE(MAX_TREE_NODES)];

	U32 count;
} PS_END_ALIGNED(PS_SIMD_FLEN);


/*!
 * Matrices in primitive are aggregated matrices of their branch in BlobTree.
 * Most of the primitives has identity matrices.
 */
struct PS_BEGIN_ALIGNED(PS_SIMD_FLEN) SOABlobNodeMatrices
{
	float matrix[PS_SIMD_PADSIZE(MAX_MTX_NODES * PRIM_MATRIX_STRIDE)];
	U32 count;
} PS_END_ALIGNED(PS_SIMD_FLEN);


/*!
 * Matrices to transform boxes
 */
struct PS_BEGIN_ALIGNED(PS_SIMD_FLEN) SOABlobBoxMatrices
{
	float matrix[PS_SIMD_PADSIZE(MAX_MTX_NODES * BOX_MATRIX_STRIDE)];
	U32 count;
} PS_END_ALIGNED(PS_SIMD_FLEN);



//////////////////////////////////////////////////////////
//Each cell per each MPU will at most contribute to 5 triangles
//Number of Cells are (GRIM_DIM - 1)^ 3
//SS = 3 * 512 * 3 * 4 + 1 * 512 * 3 * 2 + 5 + 4 * 4 =  21528
struct PS_BEGIN_ALIGNED(PS_SIMD_FLEN) MPU{
	U32   idxGlobalID;
	//Number of vertices and triangles in output mesh
	U16	  ctVertices;
	U16	  ctTriangles;
	//float error;

	vec3f bboxLo;
	U32 ctFieldEvals;
} PS_END_ALIGNED(PS_SIMD_FLEN);


//Global Mesh Holds Mesh Data
struct PS_BEGIN_ALIGNED(PS_SIMD_FLEN) MPUGLOBALMESH{
	//Pos, norm and Color will be written to
	float* vPos;
	float* vNorm;
	float* vColor;
	U32* vTriangles;

	//MPU vertex, triangle
	U32 mpuMeshVertexStrideF;
	U32 mpuMeshTriangleStrideU32;

	U64 globalMeshVertexBufferSizeF;
	U64 globalMeshTriangleBufferSizeU32;

	U64 memSizeInBytes;
} PS_END_ALIGNED(PS_SIMD_FLEN);


//Statistics for studying distribution of MPUs
struct MPUSTATS{
	int idxThread;
	int bIntersected;
	tbb_thread::id threadID;
	tbb::tick_count tickStart;
	tbb::tick_count tickEnd;
	tbb::tick_count tickFieldEvals;
};

//////////////////////////////////////////////////////////
//SS = 5120
class EdgeTable
{
public:
	EdgeTable();
	virtual ~EdgeTable();

	//memory management
	static U64 MemSizeInBytes(const MPUDim& mpuDim);
	void allocate(const MPUDim& mpuDim);
	void cleanup();

	void reset(const MPUDim& mpuDim);

	//access to edges
	int getEdge(int i, int j, int k, int edgeAxis) const;
	void setEdge(int i, int j, int k, int edgeAxis, int vid) const;

	int getEdge(vec3i& start, vec3i& end) const;
	void setEdge(vec3i& start, vec3i& end, int vid) const;

private:
	void init();

public:
	//Check ctEdges to see if there is an edge associated with vertex s
	U8* m_pEdgesCount;

	//1 if corner (i,j,k) has edge with corners i+1, j+1, k+1
	U8* m_pHasEdgeWithHiNeighbor;

	//Each internal vertex is connected to at most 6 others
	//Each internal vertex has at most 3 adjacent vertices with higher address
	U16* m_pIndexVertices;

	//MPU dimension
	MPUDim m_mpuDim;

//	//Check ctEdges to see if there is an edge associated with vertex s
//	U8 ctEdges[GRID_DIM * GRID_DIM * GRID_DIM];
//
//	//1 if corner (i,j,k) has edge with corners i+1, j+1, k+1
//	U8 hasEdgeWithHiNeighbor[GRID_DIM * GRID_DIM * GRID_DIM * 3];
//
//	//Each internal vertex is connected to at most 6 others
//	//Each internal vertex has at most 3 adjacent vertices with higher address
//	U16 idxVertices[GRID_DIM * GRID_DIM * GRID_DIM * 3];
};

//////////////////////////////////////////////////////////
template<int _size>
struct SIMPLESTACK
{
	SIMPLESTACK() { count = 0;}
	void push(U32 v)
	{
		assert(count < _size);
		id[count] = v;
		count++;
	}

	void pop() {count--;}
	bool empty() { return (count == 0);}
	void clear() { count = 0;}
	U32 top() {return id[count - 1];}

	U32 id[_size];
	int count;
};

template<typename T, int _size>
struct GENERIC_PAIR_STACK
{
	GENERIC_PAIR_STACK() { count = 0;}
	void push(T first_, T second_)
	{
		assert(count < _size);
		arrFirstVals[count] = first_;
		arrSecondVals[count] = second_;
		count++;
	}

	void pop() {count--;}
	bool empty() { return (count == 0);}
	T topFirst() {return arrFirstVals[count - 1];}
	T topSecond() {return arrSecondVals[count - 1];}
	void clear() { count =0;}

	//Members
	T arrFirstVals[_size];
	T arrSecondVals[_size];
	int count;
};

//Polymorphic Object
template<typename T, int _size>
struct GENERIC_POLYMORPHIC_STACK
{
	GENERIC_POLYMORPHIC_STACK() { count = 0;}
	void push(const T& first_)
	{
		assert(count < _size);
		arrFirstVals[count] = first_;
		count++;
	}

	void pop() {count--;}
	bool empty() { return (count == 0);}
	T top() {return arrFirstVals[count - 1];}

	//Members
	T arrFirstVals[_size];
	int count;
};


//////////////////////////////////////////////////////////

/*!
 * Computes fieldvalue, normal and gradient in the course of polygonization
 */
class FieldComputer
{
public:
	FieldComputer() {}
	FieldComputer(SOABlobPrims* lpPrims,
				  SOABlobBoxMatrices* lpPrimMatrices,
				  SOABlobOps* lpOps);

	void setup(SOABlobPrims* lpPrims,
			   SOABlobBoxMatrices* lpPrimMatrices,
			   SOABlobOps* lpOps);

	/*!
	 * Computes field-value using SIMD instructions for SIMD Length number of points.
	 * Can store field due to every node in the BlobTree.
	 * @param pX x coordinate for all points in SIMD
	 * @param pY y coordinate for all points in SIMD
	 * @param pZ z coordinate for all points in SIMD
	 * @param outField output field values for all points
	 * @param lpOutFieldPrims optional output for storing field for all primitives
	 * @param lpOutFieldOps optional output for storing field for all operators
	 */
	int fieldValue(const Float_& pX, const Float_& pY, const Float_& pZ,
				   Float_& outField, U32 idxRootNode = 0,
				   float* lpOutFieldPrims = NULL, float* lpOutFieldOps = NULL) const;

	int normal(const Float_& pX, const Float_& pY, const Float_& pZ,
			   const Float_& inFieldValue,
			   Float_& outNormalX, Float_& outNormalY, Float_& outNormalZ,
			   float delta) const;

	int gradient(const Float_& pX, const Float_& pY, const Float_& pZ,
	 		     const Float_& inFieldValue,
				 Float_& outGradX, Float_& outGradY, Float_& outGradZ,
				 float delta) const;

	//Compute field and color
	int fieldValueAndColor(const Float_& pX, const Float_& pY, const Float_& pZ,
						   Float_& outField, Float_& outColorX, Float_& outColorY, Float_& outColorZ, U32 idxRootNode = 0) const;

	int fieldValueAndGradient(const Float_& pX, const Float_& pY, const Float_& pZ,
						     Float_& outField, Float_& outGradX, Float_& outGradY, Float_& outGradZ,
							float delta) const;

	int computeRootNewtonRaphson(const vec3f& p1, const vec3f& p2,
								 float fp1, float fp2,
								 vec3f& output, float& outputField,
								 float target_field, int iterations) const;

	//Compute Root SIMD-Way
	bool computeRootSimd(const Float_& pX, const Float_& pY, const Float_& pZ, vec3f& outRoot) const;
	int getCrossedFaces(const float arrFields[8], int outFaces[6]) const;

public:
	static const U32 m_szConstFieldPadded = PS_SIMD_PADSIZE(MAX_TREE_NODES*PS_SIMD_FLEN);
	SOABlobPrims m_blobPrims;
	SOABlobNodeMatrices m_blobPrimMatrices;
	SOABlobOps 	 m_blobOps;

private:
	void computePrimitiveField(const Float_& pX, const Float_& pY, const Float_& pZ,
							   Float_& primField, U32 idxPrimitive) const;

	void computeInstancedNodeFieldAndColor(const Float_& pX, const Float_& pY, const Float_& pZ, Float_& outField,
										   Float_& outColorX, Float_& outColorY, Float_& outColorZ, U32 idxInstanceNode) const;

	//void computeColorOp

	//Check weather a given SIMD Point is outside an operator BBOX or not
	bool isOutsideOp(int idxOp, const Float_& pX, const Float_& pY, const Float_& pZ) const;

	//Check weather a given SIMD Point is outside a primitive BBOX or not
	bool isOutsidePrim(int idxNode, const Float_& pX, const Float_& pY, const Float_& pZ) const;
};

/*!
 * @brief We need this task schedular observer to create per thread BlobTree data
 */
class ThreadStartSetup : public tbb::task_scheduler_observer
{
public:
	ThreadStartSetup(const MPUDim& mpuDim,
					 const SOABlobOps* lpOps,
					 const SOABlobPrims* lpPrims,
			  	  	 const SOABlobNodeMatrices* lpMatrices);


	void on_scheduler_entry(bool is_worker);

	void on_scheduler_exit(bool is_worker);
private:
	SOABlobOps* 	m_lpBlobOps;
	SOABlobPrims* 	m_lpBlobPrims;
	SOABlobNodeMatrices* m_lpBlobMatrices;
	MPUDim m_mpuDim;
};


/*!
 * @brief Body code for parallel processing of MPUs on multi-core
 */
class CMPUProcessor
{
public:
	CMPUProcessor(float cellsize, bool bScalarRun,
				  U8 mpuDim, U32 ctMPUs, MPU* lpMPU,
				  MPUGLOBALMESH* lpGlobalMesh, MPUSTATS* lpStats);

	void operator()(const blocked_range<size_t>& range) const;

	void process_cells_simd(EdgeTable& edgeTable, const FieldComputer& fc,
							MPU& mpu, MPUGLOBALMESH& globalMesh, tbb::tick_count& tickFieldEvals) const;

	void process_cells_scalar(EdgeTable& edgeTable, const FieldComputer& fc,
							  MPU& mpu, MPUGLOBALMESH& globalMesh, tbb::tick_count& tickFieldEvals) const;
	//void process_cells_fieldsimd_cellparallel_oldedgeAccess(const FieldComputer& fc, MPU& mpu, tbb::tick_count& tickFieldEvals) const;
	//void process_cells_fieldsimd_cellserial_newedgeAccess(const FieldComputer& fc, MPU& mpu, tbb::tick_count& tickFieldEvals) const;
	//void process_cells_continuation(const FieldComputer& fc, MPU& mpu) const;

private:

	/*!
	 * Get a vertex on an edge with the indices of one corner and its axis
	 * (0=x,1=y and 2=z)
	 */
	int getEdge(int i, int j, int k, int edgeAxis) const;
	void setEdge(int i, int j, int k, int edgeAxis, int vid) const;

	int getEdge(vec3i& start, vec3i& end) const;
	void setEdge(vec3i& start, vec3i& end, int vid) const;


	bool discard(const FieldComputer& fc, MPU& mpu) const;
private:
	MPU* m_lpMPU;

	MPUSTATS* m_lpStats;
	MPUGLOBALMESH* m_lpGlobalMesh;
	MPUDim m_mpuDim;


	size_t m_ctMPUs;
	float m_cellsize;
	bool m_bScalarRun;
};


//////////////////////////////////////////////////////////
/**
 * @brief Checks BlobTree and reports all errors. First checks if all primitive bboxes are correct.
 * another check is made on operator bboxes and if all operators can contain their primitives.
 * Checks each operator being referenced only once.
 * @param blobOps input BlobTree operators
 * @param blobPrims input BlobTree primitives
 * @return number of errors detected with this BlobTree
 */
int CheckForBlobTreeErrors(SOABlobOps& blobOps, SOABlobPrims& blobPrims, bool bCheckBoxes);

/**
 * @brief prepares all primitive bounding boxes.
 * @param blobPrims input BlobTree primitives
 * @param blobOps input BlobTree operators
 * @param boxMatrices input BlobTree box matrices (Forward transformation matrix)
 * @return error code or success
 */
int PrepareAllBoxes(SOABlobOps& blobOps,
					SOABlobPrims& blobPrims,
					SOABlobBoxMatrices& boxMatrices);


int PrepareAllPrimBBoxes(SOABlobPrims& blobPrims,
				  		 SOABlobBoxMatrices& boxMatrices);

int PrepareOpBBox_Recursive(U32 idxOp,
							SOABlobOps& blobOps,
							SOABlobPrims& blobPrims,
						  	SOABlobBoxMatrices& boxMatrices,
						  	vec3f& boxLo, vec3f& boxHi);

//Compute BBOXES for all instanced nodes
int PrepareAllInstancedNodesBBoxes(SOABlobOps& blobOps,
								   SOABlobPrims& blobPrims,
								   SOABlobBoxMatrices& boxMatrices);


//Count all needed MPUs
vec3i CountMPUNeeded(float cellsize, const vec3f& lo, const vec3f& hi);

//Polygonize
int Polygonize(float cellsize,
			   bool bScalarRun,
			   const SOABlobOps& blobOps,
			   const SOABlobPrims& blobPrims,
			   const SOABlobNodeMatrices& blobMatrices,
			   U32 ctMPUs,
			   MPU* lpMPUs,
			   const MPUDim& mpuDim,
			   MPUGLOBALMESH* lpGlobalMesh,
			   MPUSTATS* lpProcessStats = NULL);


void PrintThreadResults(int ctAttempts, U32* lpThreadProcessed = NULL, U32* lpThreadCrossed = NULL);



inline void ComputeWyvillFieldValueSquare_(const Float_& dd, Float_& arrFields)
{
	Float_ allOne(1.0f);
	Float_ allZero(0.0f);
	Float_ res;
	res = (allOne - dd);
	arrFields = (res * res) * res;

	//Field less than zero is undefined. in case distance is greater than one
	arrFields = SimdMax(allZero, arrFields);
}

//Handy Functions
inline float ComputeWyvillFieldValueSquare(float dd)
{
	if(dd >= 1.0f)
		return 0.0f;
	float t = (1.0f - dd);
	return t*t*t;
}

//Returns the distance to the skeleton
inline float ComputeWyvillIsoDistance(float fv)
{
	if(fv == 0.0f) return 1.0f;
	float oneThird = 1.0f / 3.0f;
	return sqrt(1.0f - pow(fv, oneThird));
}

inline bool BoxIntersect(const vec3f& lo1, const vec3f& hi1,
   					     const vec3f& lo2, const vec3f& hi2)
{
	if ((lo1.x >= hi2.x) || (hi1.x <= lo2.x))
		return false;
	if ((lo1.y >= hi2.y) || (hi1.y <= lo2.y))
		return false;
	if ((lo1.z >= hi2.z) || (hi1.z <= lo2.z))
		return false;

	return true;
}


}
}


#endif

