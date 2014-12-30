//OpenCL BlobTree Polygonizer: Pourya Shirazian

#define DATASIZE_HEADER		12
#define DATASIZE_OPERATOR	8
#define DATASIZE_PRIMITIVE	20

#define DATASIZE_OPERATOR_F4	DATASIZE_OPERATOR/4
#define DATASIZE_PRIMITIVE_F4	DATASIZE_PRIMITIVE/4

//OFFSETS in PRIMITIVES
#define OFFSET4_PRIM_TYPE		0
#define OFFSET4_PRIM_POS		1
#define OFFSET4_PRIM_DIR		2
#define OFFSET4_PRIM_RES		3
#define OFFSET4_PRIM_COLOR		4

//OFFSETS in OPERATORS
#define OFFSET4_OP_TYPE			0
#define OFFSET4_OP_RES			1

#define OFFSET_OP_TYPE			0
#define OFFSET_OP_FLAGS			1
#define OFFSET_OP_LCRC			2
#define OFFSET_OP_PARENTLINK	3


//OFFSETS in HEADER
#define OFFSET4_HEADER_LOWER	0
#define OFFSET4_HEADER_UPPER	1
#define OFFSET_HEADER_COUNT_PRIMS 	8
#define OFFSET_HEADER_COUNT_OPS 	9
#define OFFSET_HEADER_COUNT_MTX		10
#define OFFSET_HEADER_CELLSIZE 		11

//BlobTree
#define MAX_TREE_DEPTH	 64
#define MAX_TREE_NODES   1024
#define MAX_MTX_NODES MAX_TREE_NODES * 2
#define PRIM_MATRIX_STRIDE 12
#define BOX_MATRIX_STRIDE 16

#define ISO_VALUE 0.5f

#define ZERO4 {0.0f, 0.0f, 0.0f, 0.0f}
#define AXISX {1.0f, 0.0f, 0.0f, 0.0f}
#define AXISY {0.0f, 1.0f, 0.0f, 0.0f}
#define AXISZ {0.0f, 0.0f, 1.0f, 0.0f}

//Sampler
sampler_t tableSampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;

//Comfort Types
typedef unsigned char		U8;
typedef unsigned short		U16;
typedef unsigned int		U32;
typedef			 char		I8;
typedef			 short		I16;
typedef			 int		I32;


//Types
enum PrimitiveType {primPoint, primLine, primCylinder, primDisc, primRing, primCube, 
					primTriangle, primQuadricPoint, primNULL, primInstance};
					
enum OperatorType {opUnion, opIntersect, opDif, opSmoothDif, opBlend, opRicciBlend, 
				   opGradientBlend, opFastQuadricPointSet, opCache, opWarpTwist, 
				   opWarpTaper, opWarpBend, opWarpShear};
				   
enum OPFLAGS {ofRightChildIsOp = 1, ofLeftChildIsOp = 2, ofChildIndexIsRange = 4, ofIsUnaryOp = 8};


//Stack
typedef struct{
	U16 arrIndices[MAX_TREE_DEPTH];
	int idxTop;
}STACKU16;

typedef struct{
	U16 arrIndices[MAX_TREE_DEPTH];
	float arrFields[MAX_TREE_DEPTH];
	int idxTop;
}STACKPAIR;

void stkPush(U16 idxOp, STACKU16* stkOps)
{
	stkOps->idxTop++;
	stkOps->arrIndices[stkOps->idxTop] = idxOp;
}

void stkPop(STACKU16* stkOps)
{	
	stkOps->idxTop--;
}

void stkPushPair(U16 idxOp, float field, STACKPAIR* stkOpsField)
{
	stkOpsField->idxTop++;
	stkOpsField->arrIndices[stkOpsField->idxTop] = idxOp;
	stkOpsField->arrFields[stkOpsField->idxTop] = field;
}

void stkPopPair(STACKPAIR* stkOpsField)
{	
	stkOpsField->idxTop--;
}


//Kernel Function to compute dist2ances to sphere at origin 
__kernel void poly(__global float4* arrMeshVertex,
				   __global float4* arrMeshColor,
				   __global float* arrPosX, 
				   __global float* arrPosY, 
				   __global float* arrPosZ,				   
				   const unsigned int count)
{	
	int idx = get_global_id(0);
	if(idx < count)
	{	
		float4 v = (float4)(arrPosX[idx], arrPosY[idx], arrPosZ[idx], 1.0);		
		arrMeshVertex[idx] = v;
		float4 c = 255.0f * normalize(v);
		arrMeshColor[idx] = (float4)(c.x, c.y, c.z, 1.0f); 				
	}	
};


//Computes Wyvill Field-Function
float ComputeWyvillField(float dd)
{
	if(dd >= 1.0f)
		return 0.0f;
	float t = (1.0f - dd);
	return t*t*t;
}

/*!
 * Compute field due to a primitive.
 */
float ComputePrimitiveField(int idxPrimitive,
							float4 v,			
							__global float4* arrOps4,
							__global float4* arrPrims4, 
							__global float4* arrMtxNodes4)
{
	//Transform
	int primType  = (int)(arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4].x);		
	float4 vt = v;
	
	{
		int idxMatrix = (int)(arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4].y) & 0xFFFF;		
		float4 row0 = arrMtxNodes4[idxMatrix * 3];
		float4 row1 = arrMtxNodes4[idxMatrix * 3 + 1];
		float4 row2 = arrMtxNodes4[idxMatrix * 3 + 2];
		vt = (float4)(dot(row0, v), dot(row1, v), dot(row2, v), 0.0f);
	}
		

	float dist2;

	//Compute distance to the skeletal primitive
	switch(primType)
	{
	case(primPoint):
	{
		float4 d2 = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_POS] - vt;
		dist2 = dot(d2, d2);		
	}	
	break;
	case(primLine):
	{
		float4 line0 = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_POS];
		float4 lineDelta = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_DIR] - line0;
		float lineDeltaDot = dot(lineDelta, lineDelta);
		float delta = dot(vt - line0, lineDelta) / lineDeltaDot;
		float4 tt = vt - (line0 + delta * lineDelta);
		dist2 = dot(tt, tt);
	}
	break;
	case(primCylinder):
	{
		float4 pos = vt - arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_POS];
		float4 dir = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_DIR];
		float radius = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].x;
		float height = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].y;
		float y = dot(pos, dir);
		float x = fmax(0.0f, sqrt(dot(pos, pos) - y*y) - radius);
		if(isgreater(y, 0.0f))
			y = fmax(0.0f, y - height);
		dist2 = x*x + y*y;			
	}	
	break;	
	case(primTriangle):
	{
		dist2  = 10.0f;
	}
	break;	

	case(primCube):
	{
		float4 dif = vt - arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_POS];
		float side = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].x;
				
		float projected;
		float delta;
			
		//Along X
		float4 axis = (float4)(1.0f, 0.0f, 0.0f, 0.0f);		
		projected = dot(dif, axis);
		if(isless(projected, -1.0f * side))
		{
		    delta = projected + side;
		    dist2 += delta*delta;
		}
		else if (isgreater(projected, side))
		{
		    delta = projected - side;
		    dist2 += delta*delta;
		}

		//Along Y
		axis = (float4)(0.0f, 1.0f, 0.0f, 0.0f);
		projected = dot(dif, axis);
		if(isless(projected, -1.0f * side))
		{
		    delta = projected + side;
		    dist2 += delta*delta;
		}
		else if (isgreater(projected, side))
		{
		    delta = projected - side;
		    dist2 += delta*delta;
		}

		//Along Z
		axis = (float4)(0.0f, 0.0f, 1.0f, 0.0f);
		projected = dot(dif, axis);
		if(isless(projected, -1.0f * side))
		{
		    delta = projected + side;
		    dist2 += delta*delta;
		}
		else if (isgreater(projected, side))
		{
		    delta = projected - side;
		    dist2 += delta*delta;
		}		
	}
	break;
	
	case(primDisc):
	{
		float4 delta = vt - arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_POS];
		float4 n = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_DIR];
		float r = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].x;
		float4 dir = delta - dot(delta, n) * n;
		float len2 = dot(dir, dir);
		if(islessequal(len2, r*r))
			dist2 = fabs(dot(delta, delta) - len2);
		else
		{	
			dir = normalize(dir);
			float4 x = r*dir - delta;
			dist2 = dot(x, x);
		}			
	}
	break;
	case(primRing):
	{
		float4 delta = vt - arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_POS];
		float4 n   = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_DIR];
		float r    = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].x;
		float4 dir = delta - dot(delta, n) * n;

		if(isequal(length(dir), 0.0f))
			dist2 = r*r + dot(delta, delta);
		else
		{	
			dir = normalize(dir);				
			float4 x = r*dir - delta;
			dist2 = dot(x, x);
		}			
	}
	break;
	
	case(primQuadricPoint):
	{
		float4 dt = vt - arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_POS];
		float rs  = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_DIR].z;
		float cf1 = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].x;
		float cf2 = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].y;
		float cf3 = arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].z;
		dist2 = dot(dt, dt);
		if(isgreater(rs, dist2))
			return dist2*dist2*cf1 + dist2*cf2 + cf3;
	}
	break;
	case(primNULL):
	{
		dist2 = 10.0f;
	}
	break;
	
	case(primInstance):
	{
		int idxOrigin = (int)arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].x;
		int isOriginOp = (int)arrPrims4[idxPrimitive * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_RES].z;
		//return ComputeOperatorField(idxOrigin, vt, arrOps4, arrPrims4, arrMtxNodes4);		
	}
	break;
		
	}

 	return ComputeWyvillField(dist2); 
}


/*!
* The input BlobTree is arranged as the following: HEADER + PRIMS + OPSS
* Header is 12 floats: 4F Lower BBOX + 4F Upper BBOX + (ctPrims + ctOps + ctMtxNodes + cellsize)
* Prims is 20 floats per each: type + idxMatrix + 4F Pos + 4F Dir + 4F Res + 4F Color
* Ops is 8 floats per each: type + opFlags + opLeftRightChildrenIds + 4F res
* Main Kernel to compute cell configurations.
* @param arrInHeader the header of the BlobTree input model
* @param arrInPrims the primitives input BlobTree
* @param arrInMtxNode the matrices of the input BlobTree
*/
float ComputeOperatorField(int idxOperator,
						  float4 v,						
						  __global float* arrInHeader,
						  __global float* arrInOps,
						  __global float* arrInPrims,						  
						  __global float* arrInMtxNode)
{
	//Access vector4
	__global float4* header4  = (__global float4 *)arrInHeader;
	__global float4* ops4     = (__global float4 *)arrInOps;
	__global float4* prims4   = (__global float4 *)arrInPrims;	
	__global float4* matrix4  = (__global float4 *)arrInMtxNode;

	//Compute Point		
	float4 resColor = (float4)(0.0f, 1.0f, 0.0f, 1.0f);
	float resField  = 0.0f;
	
	//Stack for operators and intermediate field values
	STACKU16 stkOps;
	STACKPAIR stkOpsField;
	stkOps.idxTop = -1;
	stkOpsField.idxTop = -1;
	
	//Push first operator onto stack
	if((int)arrInHeader[OFFSET_HEADER_COUNT_OPS] > 0)
		stkPush(0, &stkOps); 
	else
	{
		int ctPrims = (int)arrInHeader[OFFSET_HEADER_COUNT_PRIMS];		 
		resColor = (float4)(0.0f, 0.0f, 0.0f, 0.0f);		
		for(int i=0;i<ctPrims;i++)
		{
			float curField = ComputePrimitiveField(i, v, ops4, prims4, matrix4);
			resColor += curField * prims4[i * DATASIZE_PRIMITIVE_F4 + OFFSET4_PRIM_COLOR];
			resField += curField; 
		}
	}


	//While Stack is non-empty
	while(stkOps.idxTop != -1)
	{
		//Get top of stack op Index
		int idxOp = stkOps.arrIndices[stkOps.idxTop];
				
		//2. Fetch op flags
		//Check children types
		//bool isUnary = (((int)(ops4[idxOp * DATASIZE_OPERATOR_F4].y) & 0x08) != 0);
		//bool isRange = (((int)(ops4[idxOp * DATASIZE_OPERATOR_F4].y) & 0x04) != 0);		
		bool isLCOp = (((int)(ops4[idxOp * DATASIZE_OPERATOR_F4].y) & 0x02) != 0);
		bool isRCOp = (((int)(ops4[idxOp * DATASIZE_OPERATOR_F4].y) & 0x01) != 0);

		//3. Fetch left, right children indices
		U16 idxLC = ((int)(ops4[idxOp * DATASIZE_OPERATOR_F4].z) >> 16) & 0xFFFF;	
		U16 idxRC = ((int)(ops4[idxOp * DATASIZE_OPERATOR_F4].z)) & 0xFFFF;	
		U8 ctAvailable = 0;
		float lcf, rcf;
		
		//Left Child
		if(isLCOp)
		{
			//If Field Stack is Empty
			if(stkOpsField.idxTop == -1)
				stkPush(idxLC, &stkOps);
			//Else if the top of field stack belongs to this child
			else if(stkOpsField.arrIndices[stkOpsField.idxTop] == idxLC)
			{
				lcf = stkOpsField.arrFields[stkOpsField.idxTop];
				stkPopPair(&stkOpsField);
				ctAvailable++;
			}
			//Else push onto stack
			else
				stkPush(idxLC, &stkOps);
		}
		else
			ctAvailable++;


		//Right Child
		if(isRCOp)
		{
			//If Field Stack is Empty
			if(stkOpsField.idxTop == -1)
				stkPush(idxRC, &stkOps);
			//Else if the top of field stack belongs to this child
			else if(stkOpsField.arrIndices[stkOpsField.idxTop] == idxRC)
			{
				rcf = stkOpsField.arrFields[stkOpsField.idxTop];
				stkPopPair(&stkOpsField);
				ctAvailable++;
			}
			//Else push onto stack
			else
				stkPush(idxRC, &stkOps);
		}
		else
			ctAvailable++;

		//Evaluate the field for this operator and store it in the op field stack
		if(ctAvailable >= 2)
		{
			//Compute Primitive Fields
			if(isLCOp == 0)
				lcf = ComputePrimitiveField(idxLC, v, ops4, prims4, matrix4);
			if(isRCOp == 0)
				rcf = ComputePrimitiveField(idxRC, v, ops4, prims4, matrix4);

			//1. Fetch op type
			U16 opType = (int)(ops4[idxOp * DATASIZE_OPERATOR_F4].x);			
			switch(opType)
			{
			case(opBlend):
				resField = lcf + rcf;
					//	Float_ lcf  = two * (half + leftChildField) - one;
					//	Float_ rcf  = two * (half + rightChildField) - one;
				break;
			}

			//Push the pair on to stack
			stkPushPair(idxOp, resField, &stkOpsField);
			stkPop(&stkOps);
		}
	}

	return resField;
}

/*!
* Compute Cell Configuration index and Number Vertices Output for all cells.
*/
__kernel void ComputeConfigIndexVertexCount(const uint ctTotalCells,
											__global uchar* arrOutCellConfig,
											__global uchar* arrOutVertexCount,
											__global float* arrInHeader,
											__global float* arrInOps,										 
											__global float* arrInPrims,											 
											 __global float* arrInMtxNode,
											 const __global uint* arrInNeededCells,
											 __read_only image2d_t texInVertexCountTable)
{
	//Get XY plane index
	int idX = get_global_id(0);
	int idY = get_global_id(1);
	int idZ = get_global_id(2);
	if((idX >= arrInNeededCells[0])||(idY >= arrInNeededCells[1])||(idZ >= arrInNeededCells[2]))
		return;
	uint idxCell = idZ * (arrInNeededCells[0] * arrInNeededCells[1]) + idY * arrInNeededCells[0] + idX;
	if(idxCell > ctTotalCells)
		return;

	__global float4* header4  = (__global float4 *)arrInHeader;

	const float cellsize = arrInHeader[OFFSET_HEADER_CELLSIZE];
	
	float4 lower = header4[0] + cellsize * (float4)(idX, idY, idZ, 0.0f);
	float arrFields[8];
	float4 arrVertices[8];

	arrVertices[0] = lower;
	arrVertices[1] = lower + cellsize * (int4)(1, 0, 0, 0);
	arrVertices[2] = lower + cellsize * (int4)(1, 1, 0, 0);
  	arrVertices[3] = lower + cellsize * (int4)(0, 1, 0, 0);
	arrVertices[4] = lower + cellsize * (int4)(0, 0, 1, 0);
	arrVertices[5] = lower + cellsize * (int4)(1, 0, 1, 0);
  	arrVertices[6] = lower + cellsize * (int4)(1, 1, 1, 0);
	arrVertices[7] = lower + cellsize * (int4)(0, 1, 1, 0);

	for(int i=0; i<8; i++)
		arrFields[i] = ComputeOperatorField(0, arrVertices[i], arrInHeader, arrInOps, arrInPrims, arrInMtxNode);
	
    // calculate flag indicating if each vertex is inside or outside isosurface
    int idxConfig;
	idxConfig =  (arrFields[0] >= ISO_VALUE); 
	idxConfig += (arrFields[1] >= ISO_VALUE)*2; 
	idxConfig += (arrFields[2] >= ISO_VALUE)*4; 
	idxConfig += (arrFields[3] >= ISO_VALUE)*8; 
	idxConfig += (arrFields[4] >= ISO_VALUE)*16; 
	idxConfig += (arrFields[5] >= ISO_VALUE)*32; 
	idxConfig += (arrFields[6] >= ISO_VALUE)*64; 
	idxConfig += (arrFields[7] >= ISO_VALUE)*128;

    // read number of vertices from texture
    U8 ctVertices = read_imageui(texInVertexCountTable, tableSampler, (int2)(idxConfig,0)).x;

	arrOutCellConfig[idxCell] = idxConfig;
	arrOutVertexCount[idxCell] = ctVertices;
}