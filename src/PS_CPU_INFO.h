/*
 * PS_CPU_INFO.h
 *
 *  Created on: 2011-10-18
 *      Author: pourya
 */

#ifndef PS_CPU_INFO_H_
#define PS_CPU_INFO_H_

#include <stddef.h>
#include "PS_MathBase.h"

#define MAX_CACHE_INFO_LENGTH 5
#define MAX_CACHE_LEVEL 3

enum CACHE_TYPE {ctNone, ctData, ctInstruction, ctUnified};

class ProcessorInfo{
public:
	
	ProcessorInfo() { getAllInfo();}
	~ProcessorInfo() {}

	void getAllInfo();
	void getLevelInfo(int level);

	/*!
	 * Return the last level Cache size in bytes
	 */
	U32 getLastLevelSize();

#ifdef PS_OS_LINUX
	static int GetCacheNodeIndex(int level, CACHE_TYPE cacheType = ctUnified);
#endif

	/*!
	 * Returns the type of cache in a string format.
	 */
	static const char* GetCacheTypeString(CACHE_TYPE t);

	/*!
	 * Gets the line size for cache
	 */
	static U32 GetCacheLineSize();

	/*!
	 * Returns true if a specified cache level exist
	 */
	static bool HasCacheLevel(int level, CACHE_TYPE cacheType = ctUnified);

	/*!
	 * Gets cache size.
	 * @param level cache
	 * @return cache size in Bytes
	 */
	static U32 GetCacheSize(int level, CACHE_TYPE cacheType = ctUnified);

	/*!
	 * Get the type of cache
	 */
	static CACHE_TYPE GetCacheType(int level, CACHE_TYPE cacheType = ctUnified);

public:

	U8 ctCores;
	U8 simd_float_lines;
	bool bSupportAVX;
	bool bSupportSSE;
	bool bOSSupportAVX;

	U8  ctCacheInfo;
	U8  cache_line_size;
	U8  cache_levels[MAX_CACHE_INFO_LENGTH];
	U32 cache_sizes[MAX_CACHE_INFO_LENGTH];
	CACHE_TYPE cache_types[MAX_CACHE_INFO_LENGTH];
};



#endif /* PS_CPU_INFO_H_ */
