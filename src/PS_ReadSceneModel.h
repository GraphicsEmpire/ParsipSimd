/*
 * PS_ReadSceneModel.h
 *
 *  Created on: 2011-10-19
 *      Author: pourya
 */

#ifndef PS_READSCENEMODEL_H_
#define PS_READSCENEMODEL_H_

#include "AA_Models/PS_SketchConfig.h"
#include "AA_Models/PS_FileDirectory.h"
#include "PS_PolyMemManager.h"

using namespace PS::MATHSIMD;
using namespace PS::SIMDPOLY;

#define ERR_INVALID_MODEL_FILE 	 -1
#define ERR_INVALID_LAYERS_COUNT -2
#define ERR_NODE_OVERFLOW 		 -3
#define ERR_INVALID_ROOT_ID 	 -4
#define ERR_READ_MODEL_HAS_PROBLEMS -5
#define ERR_UNKNOWN_PRIM -6
#define ERR_UNKNOWN_OP 	 -7
#define MODELREAD_SUCCESS 1
#define MIN_FILE_VERSION  6

bool ExportMeshAsObj(const char* lpFilePath, const PolyMPUs& polyMPUs);



/*!
 * Reads a model from disk
 */
class ModelReader{
public:
	/*!
	 * Constructor of ModelReader with BlobTree compact structure
	 */
	ModelReader(SOABlobPrims* lpPrims, SOABlobOps* lpOps,
				SOABlobNodeMatrices* lpMtxNodes,
				SOABlobBoxMatrices* lpMtxBox);

	/*!
	 * Constructor using SimdPoly instance.
	 */
	ModelReader(SimdPoly& simdPoly);
	virtual ~ModelReader();

	int read(const char* lpFilePath);

	/*!
	 * Returns BlobTree opcode for the specified operator
	 */
	static U8 GetScriptOpType(const char* chrOpName);

	/*!
	 * Returns BlobTree primcode for the specified primitive
	 */
	static U8 GetScriptPrimType(const char* chrPrimName);
protected:
	int readNode(PS::CSketchConfig* lpScript, int id, int* lpOutIsOp = NULL);

	/*!
	 * Reads an affine transformation node and convert it to a 4x4 matrix
	 * @param lpScript the pointer to the script in memory
	 * @param id the node id
	 * @return the matrix node in the matrix data structure
	 */
	int readTransformation(PS::CSketchConfig* lpScript, int id);

private:
	int setAllInstancedNodes();

private:
	struct READNODES{
		U32 idxScript;
		U32 idxArray;
	};

	U32 m_ctInstancedNodes;
	std::vector<READNODES> m_lstReadNodes;

	SOABlobPrims* m_lpBlobPrims;
	SOABlobOps* m_lpBlobOps;
	SOABlobNodeMatrices* m_lpMtxNode;
	SOABlobBoxMatrices* m_lpMtxBox;
};


#endif /* PS_READSCENEMODEL_H_ */
