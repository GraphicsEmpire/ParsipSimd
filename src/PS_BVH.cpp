/*
 * PS_BVH.cpp
 *
 *  Created on: 2011-10-09
 *      Author: pourya
 */
#include <assert.h>
#include <algorithm>
#include "PS_BVH.h"



// BVHAccel Local Declarations
struct BVHPrimitiveInfo {
    BVHPrimitiveInfo() { }
    BVHPrimitiveInfo(int pn, const svec3f& lo, const svec3f& hi)
    {
    	primitiveNumber = pn;
    	boundsLo = lo;
    	boundsHi = hi;
        centroid = vscale3f(0.5f, vadd3f(lo, hi));
    }
    int primitiveNumber;
    svec3f centroid;
    svec3f boundsLo;
    svec3f boundsHi;
};


struct BVHBuildNode {
    // BVHBuildNode Public Methods
    BVHBuildNode() { children[0] = children[1] = NULL; }
    void InitLeaf(U32 first, U32 n, const svec3f &lo, const svec3f& hi) {
        firstPrimOffset = first;
        nPrimitives = n;
        boundLo = lo;
        boundHi = hi;
    }
    void InitInterior(U32 axis, BVHBuildNode *c0, BVHBuildNode *c1) {
        children[0] = c0;
        children[1] = c1;
        boundLo = vmin3f(c0->boundLo, c1->boundLo);
        boundHi = vmax3f(c0->boundHi, c1->boundHi);
        splitAxis = axis;
        nPrimitives = 0;
    }
    svec3f boundLo;
    svec3f boundHi;
    BVHBuildNode *children[2];
    U32 splitAxis, firstPrimOffset, nPrimitives;
};


struct CompareToMid {
    CompareToMid(int d, float m) { dim = d; mid = m; }
    int dim;
    float mid;
    bool operator()(const BVHPrimitiveInfo &a) const
    {
        return velement3f(a.centroid, dim) < mid;
    }
};


struct ComparePoints {
    ComparePoints(int d) { dim = d; }
    int dim;
    bool operator()(const BVHPrimitiveInfo &a,
                    const BVHPrimitiveInfo &b) const {
        return velement3f(a.centroid, dim) < velement3f(b.centroid, dim);
    }
};


struct CompareToBucket {
    CompareToBucket(int split, int num, int d, const svec3f& lo, const svec3f& hi)

    {
    	splitBucket = split; nBuckets = num; dim = d;
    	centroidBoundsLo = lo;
    	centroidBoundsHi = hi;
    }
    bool operator()(const BVHPrimitiveInfo &p) const;

    int splitBucket, nBuckets, dim;
    svec3f centroidBoundsLo;
    svec3f centroidBoundsHi;
};


bool CompareToBucket::operator()(const BVHPrimitiveInfo &p) const {
    int b = nBuckets * ((velement3f(p.centroid, dim) - velement3f(centroidBoundsLo, dim)) /
						(velement3f(centroidBoundsHi, dim) - velement3f(centroidBoundsLo, dim)));
    if (b == nBuckets) b = nBuckets-1;
    assert(b >= 0 && b < nBuckets);
    return (b <= splitBucket);
}

/*
struct LinearBVHNode {
    BBox bounds;
    union {
        uint32_t primitivesOffset;    // leaf
        uint32_t secondChildOffset;   // interior
    };

    uint8_t nPrimitives;  // 0 -> interior node
    uint8_t axis;         // interior node: xyz
    uint8_t pad[2];       // ensure 32 byte total size
};
*/

BVHAccel::BVHAccel(const SOABlobPrims* lpPrimitives, SOALinearBVH& outLinearBVH, U32 maxPrimsInBVHLeaf, SplitMethod split)
{
	m_lpPrimitives = const_cast<SOABlobPrims*>(lpPrimitives);
	U32 ctPrims = m_lpPrimitives->count;

	m_maxPrimsInNode = MATHMIN(255, maxPrimsInBVHLeaf);
	m_splitMethod = split;

	vector<BVHPrimitiveInfo> buildData;
	buildData.reserve(ctPrims);
	for(U32 i=0; i<ctPrims; i++)
	{
		svec3f lo = svec3f(m_lpPrimitives->bboxLoX[i], m_lpPrimitives->bboxLoY[i], m_lpPrimitives->bboxLoZ[i]);
		svec3f hi = svec3f(m_lpPrimitives->bboxHiX[i], m_lpPrimitives->bboxHiY[i], m_lpPrimitives->bboxHiZ[i]);

		buildData.push_back(BVHPrimitiveInfo(i, lo, hi));
	}

	U32 ctTotalBVHNodes = 0;
	U32 ctOrderedPrims = 0;
	SOABlobPrims orderedPrims;
	BVHBuildNode* root = recursiveBuild(buildData, 0, ctPrims, &ctTotalBVHNodes, orderedPrims, &ctOrderedPrims);

	memcpy(m_lpPrimitives, &orderedPrims, sizeof(SOABlobPrims));


	//make sure BVH nodes are less than Maximum
	assert(ctTotalBVHNodes  < PS_SIMD_PADSIZE(MAX_BVH_NODES));

	//Flatten
	U32 offset = 0;
	outLinearBVH.ctNodes = ctTotalBVHNodes;
	flattenBVHTree(root, &offset, outLinearBVH);


	assert(offset == ctTotalBVHNodes);
}

BVHBuildNode* BVHAccel::recursiveBuild(vector<BVHPrimitiveInfo>& buildData,
									   U32 start, U32 end, U32* ctTotalBVHNodes, SOABlobPrims& orderedPrims, U32* ctOrderedPrims)
{
	assert(start != end);
    (*ctTotalBVHNodes)++;

    BVHBuildNode *node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    svec3f bboxLo = svec3f(FLT_MAX, FLT_MAX, FLT_MAX);
    svec3f bboxHi = svec3f(FLT_MIN, FLT_MIN, FLT_MIN);
    for (U32 i = start; i < end; ++i)
    {
    	bboxLo = vmin3f(bboxLo, buildData[i].boundsLo);
    	bboxHi = vmax3f(bboxHi, buildData[i].boundsHi);
    }

    U32 nPrimitives = end - start;
    if (nPrimitives == 1)
    {
        // Create leaf _BVHBuildNode_
        U32 firstPrimOffset = (*ctOrderedPrims);
        for (U32 i = start; i < end; ++i)
        {
            U32 primNum = buildData[i].primitiveNumber;
            copyOnePrimitive(*m_lpPrimitives, orderedPrims, primNum, *ctOrderedPrims);
            (*ctOrderedPrims)++;
        }
        node->InitLeaf(firstPrimOffset, nPrimitives, bboxLo, bboxHi);
    }
    else {
        // Compute bound of primitive centroids, choose split dimension _dim_
        svec3f centroidBoundsLo = svec3f(FLT_MAX, FLT_MAX, FLT_MAX);
        svec3f centroidBoundsHi = svec3f(FLT_MIN, FLT_MIN, FLT_MIN);
        for (U32 i = start; i < end; ++i)
        {
            centroidBoundsLo = vmin3f(centroidBoundsLo, buildData[i].centroid);
            centroidBoundsHi = vmax3f(centroidBoundsHi, buildData[i].centroid);
        }

        int dim = vmaxExtent3f(centroidBoundsHi, centroidBoundsLo);

        //If maxextent is zero create a leaf node
        if (velement3f(centroidBoundsHi, dim) == velement3f(centroidBoundsLo, dim))
        {
            // Create leaf _BVHBuildNode_
            U32 firstPrimOffset = (*ctOrderedPrims);
            for (U32 i = start; i < end; ++i)
            {
                U32 primNum = buildData[i].primitiveNumber;
                copyOnePrimitive(*m_lpPrimitives, orderedPrims, primNum, *ctOrderedPrims);
                (*ctOrderedPrims)++;
            }

            node->InitLeaf(firstPrimOffset, nPrimitives, bboxLo, bboxHi);
            return node;
        }

        // Partition primitives into two sets and build children
        U32 mid = (start + end) / 2;
        // Partition primitives based on _splitMethod_
        switch (m_splitMethod)
        {
        case SPLIT_MIDDLE: {
            // Partition primitives through node's midpoint
            float pmid = 0.5f * (velement3f(centroidBoundsLo, dim) + velement3f(centroidBoundsHi, dim));

            BVHPrimitiveInfo *midPtr = std::partition(&buildData[start],
                                                      &buildData[end-1]+1,
                                                      CompareToMid(dim, pmid));
            mid = midPtr - &buildData[0];
            if (mid != start && mid != end)
                // for lots of prims with large overlapping bounding boxes, this
                // may fail to partition; in that case don't break and fall through
                // to SPLIT_EQUAL_COUNTS
                break;
        }
        case SPLIT_EQUAL_COUNTS: {
            // Partition primitives into equally-sized subsets
            mid = (start + end) / 2;
            std::nth_element(&buildData[start], &buildData[mid],
                             &buildData[end-1]+1, ComparePoints(dim));
            break;
        }
        case SPLIT_SAH: default: {
            // Partition primitives using approximate SAH
            if (nPrimitives <= 4) {
                // Partition primitives into equally-sized subsets
                mid = (start + end) / 2;
                std::nth_element(&buildData[start], &buildData[mid],
                                 &buildData[end-1]+1, ComparePoints(dim));
            }
            else {
                // Allocate _BucketInfo_ for SAH partition buckets
                const int nBuckets = 12;
                struct BucketInfo {
                    BucketInfo() {
                    	count = 0;
                    	boundsLo = svec3f(FLT_MAX, FLT_MAX, FLT_MAX);
                    	boundsHi = svec3f(FLT_MIN, FLT_MIN, FLT_MIN);
                    }
                    int count;
                    svec3f boundsLo;
                    svec3f boundsHi;
                };
                BucketInfo buckets[nBuckets];

                // Initialize _BucketInfo_ for SAH partition buckets
                // Distribute primitives among buckets
                for (U32 i = start; i < end; ++i)
                {
                    int b = nBuckets *
                         (velement3f(buildData[i].centroid, dim) - velement3f(centroidBoundsLo, dim)) /
                         (velement3f(centroidBoundsHi, dim) - velement3f(centroidBoundsLo, dim));
                    if (b == nBuckets) b = nBuckets-1;
                    assert(b >= 0 && b < nBuckets);
                    buckets[b].count++;
                    buckets[b].boundsLo = vmin3f(buckets[b].boundsLo, buildData[i].boundsLo);
                    buckets[b].boundsHi = vmax3f(buckets[b].boundsHi, buildData[i].boundsHi);
                }

                // Compute costs for splitting after each bucket
                float cost[nBuckets-1];
                for (int i = 0; i < nBuckets-1; ++i)
                {
                	svec3f b0Hi = svec3f(FLT_MIN, FLT_MIN, FLT_MIN);
                	svec3f b1Hi = svec3f(FLT_MIN, FLT_MIN, FLT_MIN);
                	svec3f b0Lo = svec3f(FLT_MAX, FLT_MAX, FLT_MAX);
                	svec3f b1Lo = svec3f(FLT_MAX, FLT_MAX, FLT_MAX);


                    int count0 = 0, count1 = 0;
                    for (int j = 0; j <= i; ++j)
                    {
                    	b0Lo = vmin3f(b0Lo, buckets[j].boundsLo);
                    	b0Hi = vmax3f(b0Hi, buckets[j].boundsHi);

                    	//Cost of Distance computation assumed 1 per each prim
                        count0 += buckets[j].count;
                    }
                    for (int j = i+1; j < nBuckets; ++j)
                    {
                    	b1Lo = vmin3f(b1Lo, buckets[j].boundsLo);
                    	b1Hi = vmax3f(b1Hi, buckets[j].boundsHi);

                    	//Cost of Distance computation assumed 1 per each prim
                        count1 += buckets[j].count;
                    }

                    //Cost = Cost of BVH Tree Traverse +
                    //SumOver(Cost of Dist Computation for all prims in Buckets A) * (probability of Crossing A when in this BVH node) +
                    //SumOver(Cost of Dist Computation for all prims in Buckets B) * (probability of Crossing B when in this BVH node)
                    cost[i] = .125f + (count0* vsurfaceArea3f(b0Hi, b0Lo)  + count1* vsurfaceArea3f(b1Hi, b1Lo)) /
                              vsurfaceArea3f(bboxHi, bboxLo);
                }

                // Find bucket to split at that minimizes SAH metric
                float minCost = cost[0];
                U32 minCostSplit = 0;
                for (int i = 1; i < nBuckets-1; ++i) {
                    if (cost[i] < minCost) {
                        minCost = cost[i];
                        minCostSplit = i;
                    }
                }

                // Either create leaf or split primitives at selected SAH bucket

                //Splitting
                if (nPrimitives > m_maxPrimsInNode ||  minCost < nPrimitives)
                {
                    BVHPrimitiveInfo *pmid = std::partition(&buildData[start],
															&buildData[end-1]+1,
															CompareToBucket(minCostSplit, nBuckets, dim, centroidBoundsLo, centroidBoundsHi));
                    mid = pmid - &buildData[0];
                }
                else
                {
                    // Create leaf _BVHBuildNode_
                    U32 firstPrimOffset = *ctOrderedPrims;
                    for (U32 i = start; i < end; ++i)
                    {
                        U32 primNum = buildData[i].primitiveNumber;
                        copyOnePrimitive(*m_lpPrimitives, orderedPrims, primNum, *ctOrderedPrims);
                        (*ctOrderedPrims)++;
                    }

                    node->InitLeaf(firstPrimOffset, nPrimitives, bboxLo, bboxHi);
                    return node;
                }
            }
            break;
        }
        }

        node->InitInterior(dim,
                           recursiveBuild(buildData, start, mid,
                                          ctTotalBVHNodes, orderedPrims, ctOrderedPrims),
                           recursiveBuild(buildData, mid, end,
                                          ctTotalBVHNodes, orderedPrims, ctOrderedPrims));
    }
    return node;
}



U32 BVHAccel::flattenBVHTree(BVHBuildNode *node, U32 *offset, SOALinearBVH& outLinearBVH)
{
	U32 cur = *offset;
	(*offset)++;

	outLinearBVH.vBoundsLoX[cur] = node->boundLo.x;
	outLinearBVH.vBoundsLoY[cur] = node->boundLo.y;
	outLinearBVH.vBoundsLoZ[cur] = node->boundLo.z;

	outLinearBVH.vBoundsHiX[cur] = node->boundHi.x;
	outLinearBVH.vBoundsHiY[cur] = node->boundHi.y;
	outLinearBVH.vBoundsHiZ[cur] = node->boundHi.z;


	//Leaf
	if(node->nPrimitives > 0)
	{
		assert((node->children[0] == NULL) && (node->children[1] == NULL));
		outLinearBVH.uPrimitiveOffset[cur] = node->firstPrimOffset;
		outLinearBVH.ctPrimitives[cur] = node->nPrimitives;
	}
	else
	{
		outLinearBVH.splitAxis[cur] = node->splitAxis;
		outLinearBVH.ctPrimitives[cur] = 0;

		flattenBVHTree(node->children[0], offset, outLinearBVH);
		outLinearBVH.uSecondChildOffset[cur] = flattenBVHTree(node->children[1], offset, outLinearBVH);
	}

	//Free
	delete node;
	node = NULL;

	return cur;
}

void BVHAccel::copyOnePrimitive(const SOABlobPrims& from, SOABlobPrims& to, U32 idxFrom, U32 idxTo)
{
	to.bboxLoX[idxTo] = from.bboxLoX[idxFrom];
	to.bboxLoY[idxTo] = from.bboxLoY[idxFrom];
	to.bboxLoZ[idxTo] = from.bboxLoZ[idxFrom];
	to.bboxHiX[idxTo] = from.bboxHiX[idxFrom];
	to.bboxHiY[idxTo] = from.bboxHiY[idxFrom];
	to.bboxHiZ[idxTo] = from.bboxHiZ[idxFrom];

	to.posX[idxTo] = from.posX[idxFrom];
	to.posY[idxTo] = from.posY[idxFrom];
	to.posZ[idxTo] = from.posZ[idxFrom];

	to.colorX[idxTo] = from.colorX[idxFrom];
	to.colorY[idxTo] = from.colorY[idxFrom];
	to.colorZ[idxTo] = from.colorZ[idxFrom];

}





































































