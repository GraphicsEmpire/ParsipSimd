/*
 * PS_BVH.h
 *
 *  Created on: 2011-10-09
 *      Author: pourya
 */

#ifndef PS_BVH_H_
#define PS_BVH_H_

#include <vector>
#include "PS_Polygonizer.h"

using namespace std;
using namespace PS::FUNCTIONALMATH;
using namespace PS::SIMDPOLY;

#define MAX_BVH_NODES 256

struct BVHBuildNode;
struct BVHPrimitiveInfo;

//Aligned SOA Structure for Bounding Volume Hierarchies
//SS= 8*128*4 + 1*128*1 + 1 = 4225
struct PS_BEGIN_ALIGNED(PS_SIMD_FLEN) SOALinearBVH
{
	float vBoundsLoX[PS_SIMD_PADSIZE(MAX_BVH_NODES)];
	float vBoundsLoY[PS_SIMD_PADSIZE(MAX_BVH_NODES)];
	float vBoundsLoZ[PS_SIMD_PADSIZE(MAX_BVH_NODES)];

	float vBoundsHiX[PS_SIMD_PADSIZE(MAX_BVH_NODES)];
	float vBoundsHiY[PS_SIMD_PADSIZE(MAX_BVH_NODES)];
	float vBoundsHiZ[PS_SIMD_PADSIZE(MAX_BVH_NODES)];
	union{
		U32   uPrimitiveOffset[PS_SIMD_PADSIZE(MAX_BVH_NODES)]; //LEAF
		U32   uSecondChildOffset[PS_SIMD_PADSIZE(MAX_BVH_NODES)]; //Interior Nodes
	};

	//Primitive Count is zero for interior nodes
	U32	  ctPrimitives[PS_SIMD_PADSIZE(MAX_BVH_NODES)];

	U8 	  splitAxis[PS_SIMD_PADSIZE(MAX_BVH_NODES)];

	U8 ctNodes;
} PS_END_ALIGNED(PS_SIMD_FLEN);



class BVHAccel
{
public:
	enum SplitMethod { SPLIT_MIDDLE, SPLIT_EQUAL_COUNTS, SPLIT_SAH };
public:

	BVHAccel(const SOABlobPrims* lpPrimitives, SOALinearBVH& outLinearBVH, U32 maxPrimsInBVHLeaf = 4, SplitMethod split = SPLIT_SAH );


private:
	BVHBuildNode* recursiveBuild(vector<BVHPrimitiveInfo>& buildData,
								 U32 start, U32 end, U32* ctTotalBVHNodes, SOABlobPrims& orderedPrims, U32* ctOrderedPrims);

	U32 flattenBVHTree(BVHBuildNode *node, U32 *offset, SOALinearBVH& outLinearBVH);


	void copyOnePrimitive(const SOABlobPrims& from, SOABlobPrims& to, U32 idxFrom, U32 idxTo);


    SplitMethod m_splitMethod;

	SOABlobPrims* m_lpPrimitives;
	U32 m_maxPrimsInNode;
};


#endif /* PS_BVH_H_ */
