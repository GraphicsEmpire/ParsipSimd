/*
 * PS_Charts.h
 *
 *  Created on: 2012-01-25
 *      Author: pourya
 */

#ifndef PS_CHARTS_H_
#define PS_CHARTS_H_

#include "tbb/tick_count.h"
#include "implicit/PolyMemManager.h"

using namespace PS::SIMDPOLY;
using namespace tbb;

/*!
 * Get the path to the executable
 */
void GetExePath(char *exePath, int szBuffer);
bool sqlite_CreateTableIfNotExist(const char* chrDBPath, int ctProcessorCores);
bool sqlite_IsLogTableExist(const char* chrDBPath);
int sqlite_Callback(void *NotUsed, int argc, char **argv, char **azColName);

int sqlite_GetLastXPID(const char* chrDB);

bool chart_CreateStackChart(int xpID,
						    const PolyMPUs& polyMPUs,
						    MPUSTATS* lpMPUStats,
						    tick_count& t0, tick_count& t1,
						    int ctThreads);




#endif /* PS_CHARTS_H_ */
