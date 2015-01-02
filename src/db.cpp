/*
 * PS_Charts.cpp
 *
 *  Created on: 2012-01-25
 *      Author: pourya
 */
#include "sqlite/CppSQLite3.h"
#include <iostream>
#include <stdio.h>
#include <Board.h>

#include "base/FileDirectory.h"
#include "db.h"

using namespace std;
using namespace LibBoard;

using namespace PS::SIMDPOLY;

void GetExePath(char *exePath, int szBuffer)
{
#ifdef WIN32
	WCHAR wszExeName[MAX_PATH + 1];
	wszExeName[MAX_PATH] = 0;
	GetModuleFileNameW(NULL, wszExeName, sizeof(wszExeName) - 1);
	WideCharToMultiByte(CP_ACP, 0, wszExeName, -1, (LPSTR)exePath, szBuffer, NULL, NULL);
#elif defined(__linux__)
	//getcwd only retrieves the current working directory
	//getcwd(exePath, (U32)szBuffer);

	pid_t pid = getpid();
	char chrProcess[256];
	sprintf(chrProcess, "/proc/%d/exe", pid);

	int nCharsWritten = readlink(chrProcess, exePath, szBuffer);
	if(nCharsWritten != -1)
	{
		exePath[nCharsWritten] = 0;

	}
#endif
}

int sqlite_GetLastXPID(const char* chrDB)
{
	CppSQLite3DB db;
	db.open(chrDB);

	CppSQLite3Query q = db.execQuery("select MAX(xpID) from tblPerfLog");
	//for (int fld = 0; fld < q.numFields(); fld++)
	//{
		//cout << q.fieldName(fld) << "(" << q.fieldValue(fld) << ")|";
	//}
	//cout << endl;
	while(!q.eof())
	{
		if(strcmp(q.fieldName(0), "MAX(xpID)") == 0)
			return q.getIntField(0, -1);
		q.nextRow();
	}
	return -1;
}

int sqlite_Callback(void *NotUsed, int argc, char **argv, char **azColName)
{
	int i;
	for(i=0; i<argc; i++){
      printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
	}
	printf("\n");
	return 0;
}


bool sqlite_IsLogTableExist(const char* chrDBPath)
{
	sqlite3* db;
	char* lpSqlError = 0;
	int rc = sqlite3_open(chrDBPath, &db);
	if(rc)
	{
		fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
		sqlite3_close(db);
		return false;
	}

	const char *strSQL = "SELECT CASE WHEN tbl_name = 'tblPerfLog' THEN 1 ELSE 0 END FROM sqlite_master WHERE tbl_name = 'tblPerfLog' AND type = 'table'";
	rc = sqlite3_exec(db, strSQL, sqlite_Callback, 0, &lpSqlError);
	if(rc != SQLITE_OK)
	{
		fprintf(stderr, "SQL error: %s\n", lpSqlError);
		sqlite3_free(lpSqlError);
		return false;
	}
	return true;
}

bool sqlite_CreateTableIfNotExist(const char* chrDBPath, int ctProcessorCores)
{
	sqlite3* db;
	char* lpSqlError = 0;
	int rc = sqlite3_open(chrDBPath, &db);
	if(rc)
	{
		fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
		sqlite3_close(db);
		return false;
	}

	const char *strSQL1 = "CREATE TABLE IF NOT EXISTS tblPerfLog(xpID INTEGER PRIMARY KEY AUTOINCREMENT, xpModelName VARCHAR(30), xpTime DATETIME, "
						 "ctPrims int, ctOps int, mpuDim int, cellSize float, "
						 "ctGroupsTotal int, ctMPUTotal int, ctMPUIntersected int, ctTotalFieldEvals int, ctLatestMPUFieldEvals int, ctFVEPT int, "
						 "ctMeshFaces int, ctMeshVertices int, msPolyTotal double, msPolyFieldEvals double, msPolyTriangulate double, "
						 "ctThreads smallint, ctSIMDLength smallint, szWorkItemMem int, szTotalMemUsage int, szLastLevelCache int);";

	rc = sqlite3_exec(db, strSQL1, sqlite_Callback, 0, &lpSqlError);
	if(rc != SQLITE_OK)
	{
		fprintf(stderr, "SQL error: %s\n", lpSqlError);
		sqlite3_free(lpSqlError);
		return false;
	}


	std::string strSQL2 = "CREATE TABLE IF NOT EXISTS tblUtilization(id INTEGER PRIMARY KEY AUTOINCREMENT, xpID INTEGER, ctCores smallint, ";
	{
		char buffer[2048];
		for(int i=1; i<=ctProcessorCores; i++)
		{
			sprintf(buffer, "mpuCrossed%d int, mpuNonCrossed%d int, mpuCrossedTime%d double, mpuNonCrossedTime%d double, latestFinish%d double, ", i, i, i, i, i);
			strSQL2 += string(buffer);
		}
	}

	strSQL2 += "fastest int, slowest int);";
	rc = sqlite3_exec(db, strSQL2.c_str(), sqlite_Callback, 0, &lpSqlError);
	if(rc != SQLITE_OK)
	{
		fprintf(stderr, "SQL error: %s\n", lpSqlError);
		sqlite3_free(lpSqlError);
		return false;
	}

	return true;
}

bool chart_CreateStackChart(int xpID,
						    const PolyMPUs& polyMPUs,
						    MPUSTATS* lpMPUStats,
						    tick_count& t0, tick_count& t1,
						    int ctThreads)
{
	//Get All Info
	U32 statsThreadProcessed[MAX_THREADS_COUNT];
	U32 statsThreadCrossed[MAX_THREADS_COUNT];
	U32 statsThreadEmpty[MAX_THREADS_COUNT];
	double statsThreadLatestTime[MAX_THREADS_COUNT];
	double statsThreadCrossedTime[MAX_THREADS_COUNT];
	double statsThreadEmptyTime[MAX_THREADS_COUNT];
	double statsThreadFieldEvalsTime[MAX_THREADS_COUNT];
	memset(statsThreadProcessed, 0, sizeof(int) * MAX_THREADS_COUNT);
	memset(statsThreadCrossed, 0, sizeof(int) * MAX_THREADS_COUNT);
	memset(statsThreadEmpty, 0, sizeof(int) * MAX_THREADS_COUNT);

	memset(statsThreadLatestTime, 0, sizeof(double) * MAX_THREADS_COUNT);
	memset(statsThreadCrossedTime, 0, sizeof(double) * MAX_THREADS_COUNT);
	memset(statsThreadEmptyTime, 0, sizeof(double) * MAX_THREADS_COUNT);
	memset(statsThreadFieldEvalsTime, 0, sizeof(double) * MAX_THREADS_COUNT);

	int ctIntersected = 0;
	{
		std::map<tbb_thread::id, int> mapID;
		int idxThread = 0;
		int ctLastAddedThread = 0;

		U32 ctWorkUnits = polyMPUs.countWorkUnits();
		for(U32 i=0; i < ctWorkUnits; i++)
		{
			tbb_thread::id tid = lpMPUStats[i].threadID;
			if(mapID.find(tid) == mapID.end())
			{
				idxThread = ctLastAddedThread;
				mapID.insert(std::pair<tbb_thread::id, int>(tid, idxThread));
				ctLastAddedThread++;
			}
			else
				idxThread = mapID[tid];

			//Update Thread Stats
			statsThreadProcessed[idxThread]++;
			lpMPUStats[i].idxThread = idxThread;
			lpMPUStats[i].bIntersected = (polyMPUs.lpMPUs[i].ctTriangles > 0);

			//Latest time
			double et = (lpMPUStats[i].tickEnd - t0).seconds();
			if(et > statsThreadLatestTime[idxThread])
				statsThreadLatestTime[idxThread] = et;

			if(lpMPUStats[i].bIntersected)
			{
				statsThreadCrossed[idxThread]++;

				//Time to compute a crossed MPU
				statsThreadCrossedTime[idxThread] += (lpMPUStats[i].tickEnd - lpMPUStats[i].tickStart).seconds();
				ctIntersected++;
			}
			else
				statsThreadEmptyTime[idxThread] += (lpMPUStats[i].tickEnd - lpMPUStats[i].tickStart).seconds();

			//FieldEvals
			statsThreadFieldEvalsTime[idxThread] += (lpMPUStats[i].tickFieldEvals - lpMPUStats[i].tickStart).seconds();
		}
		mapID.clear();
	}

	//Index of latest and fastest
	int idxLatestThread = 0;
	int idxFastestThread = 0;
	double etLatest = statsThreadLatestTime[0];
	double etFastest = statsThreadLatestTime[0];
	for(int i=1; i<ctThreads; i++)
	{
		if(statsThreadLatestTime[i] > etLatest)
		{
			etLatest = statsThreadLatestTime[i];
			idxLatestThread = i;
		}

		if(statsThreadLatestTime[i] < etFastest)
		{
			etFastest = statsThreadLatestTime[i];
			idxFastestThread = i;
		}
	}

	//Draw to graph
	char chrFilePath[1024];
	char buffer[1024];
	{
		AnsiStr strFP = PS::FILESTRINGUTILS::ExtractFilePath(PS::FILESTRINGUTILS::GetExePath());
		strcpy(chrFilePath, strFP.cptr());
	}

	{
		double WIDTH = 800.0;
		double HEIGHT = ctThreads * 80.0;

		Board board1;
		board1.clear(LibBoard::Color(255, 255, 255));
		board1.drawRectangle(0, 0, WIDTH, HEIGHT, 1.0f);

		double wTimeUnit = WIDTH / (t1 - t0).seconds();
		double hTimeUnit = (HEIGHT - 20) / (double)ctThreads;
		double hTimeStack = 0.6 * hTimeUnit;
		double hGap = 0.2 * hTimeUnit;



		//2.Draw a quad per each MPU (Crossed Blue, Non-Crossed RED)
		U32 ctWorkUnits = polyMPUs.countWorkUnits();

		LibBoard::Color clRectPen(0,0,0);
		LibBoard::Color clRectFill;
		for(U32 i=0; i < ctWorkUnits; i++)
		{
			if(lpMPUStats[i].bIntersected)
				clRectFill = LibBoard::Color(0, 0, 255);
			else
				clRectFill = LibBoard::Color(255, 0, 0);

			double xStart = (lpMPUStats[i].tickStart - t0).seconds() * wTimeUnit;
			double yStart = -1.0f * (ctThreads - lpMPUStats[i].idxThread - 1) * hTimeUnit - hGap;
			double xWidth = (lpMPUStats[i].tickEnd - lpMPUStats[i].tickStart).seconds() * wTimeUnit;

			board1 << LibBoard::Rectangle(xStart, yStart, xWidth, hTimeStack, clRectPen, clRectFill, 0.01f);
		}

		//3. Write All Text Strings
		double hTimeStackHalf = hTimeStack * 0.1;
		vec4f black(1.0f, 1.0f, 1.0f, 1.0f);

		int avgWork = 0;
		for(int i=0; i<ctThreads; i++)
			avgWork += statsThreadProcessed[i];
		avgWork /= ctThreads;

		for(int i=0; i<ctThreads; i++)
		{
			float ratio = (statsThreadProcessed[i] != 0)? (float)(statsThreadCrossed[i] * 100.0f) / (float)statsThreadProcessed[i] : 0.0f;
			double dif = (etLatest - statsThreadLatestTime[i]) * 1000.0;
			statsThreadEmpty[i] = statsThreadProcessed[i] - statsThreadCrossed[i];
			sprintf(buffer, "Thread %d:[T %u, X %u, NX %u %.2f%], [X %.2f, NX %.2f ms], TTF %.2f ms]",
					i+1, statsThreadProcessed[i], statsThreadCrossed[i], statsThreadEmpty[i], ratio,
					statsThreadCrossedTime[i] * 1000.0, statsThreadEmptyTime[i] * 1000.0, dif);

			double y = -1.0f * (ctThreads - i - 1) * hTimeUnit - hGap/2;

			board1 << Text(20, y, string(buffer), Fonts::CourierBold, 4, LibBoard::Color::Black );
			bool bDetailed = false;
			if(bDetailed)
			{
				//Show Average Cell Processing Info
				sprintf(buffer, "Avg MPU Time: X %.2f, NX %.2f ms, ImbalanceFactor MPU %.2f",
						(float)(statsThreadCrossedTime[i] * 1000.0) /(float)statsThreadCrossed[i],
						(float)(statsThreadEmptyTime[i] * 1000.0) / (float)statsThreadEmpty[i],
						(float)(statsThreadProcessed[i]) / (float)avgWork);
				board1 << Text(20, i * hTimeUnit + hTimeStackHalf - 12, string(buffer), Fonts::CourierBold, 4, LibBoard::Color::Black );
			}
		}

		sprintf(buffer, "%s/graphs/CoreHorzUsageXP%d.eps", chrFilePath, xpID);
		board1.saveEPS(buffer, 140, ctThreads * 25.0);
		sprintf(buffer, "%s/graphs/CoreHorzUsageXP%d.svg", chrFilePath, xpID);
		board1.saveSVG(buffer, 140, ctThreads * 25.0);
	}


	//Vertical
	{
		double WIDTH = MATHMAX(100.0, 25.0 * ctThreads);
		double HEIGHT = MATHMAX(100.0, 25.0 * ctThreads);

		Board board2;
		board2.clear(LibBoard::Color(255, 255, 255));
		board2.drawRectangle(0, 0, WIDTH, HEIGHT, 1.0f);

		double hTimeUnit = HEIGHT / (t1 - t0).seconds();
		double wUnit = WIDTH / ctThreads;
		double wUnitBar = 0.6 * wUnit;
		double wGap     = 0.2 * wUnit;
		double hTotal 	= (t1 - t0).seconds() * hTimeUnit;

		for(int i=0; i<ctThreads; i++)
		{
			//FieldEval is part of Crossed
			double hFieldEvals = statsThreadFieldEvalsTime[i] * hTimeUnit;
			double hCrossed = statsThreadCrossedTime[i] * hTimeUnit;
			double hEmpty = statsThreadEmptyTime[i] * hTimeUnit;
			double hIdle = hTotal - hCrossed - hEmpty;

			double tsIdle = (t1 - t0).seconds() - statsThreadCrossedTime[i] - statsThreadEmptyTime[i];
			//double tsTTF = (t1 - t0).seconds() - statsThreadLatestTime;
			//double hTTF = tsTTF * hTimeUnit;

			//Draw Crossed in Blue
			{
				double dif = hCrossed - hFieldEvals;
				board2 << LibBoard::Rectangle(i * wUnit + wGap, -hEmpty - hIdle, wUnitBar, hCrossed, LibBoard::Color(0,0,0), LibBoard::Color(0,0,255), 0.01);
				board2 << LibBoard::Rectangle(i * wUnit + wGap, -hEmpty - hIdle - dif, wUnitBar, hFieldEvals, LibBoard::Color(0,0,0), LibBoard::Color(0,0,128), 0.01);
			}

			sprintf(buffer, "%.0f ms/%u", statsThreadCrossedTime[i] * 1000.0, statsThreadCrossed[i]);
			board2 << LibBoard::Text(i * wUnit + wGap, - hIdle - hEmpty - hCrossed / 2, string(buffer), Fonts::CourierBold, 6, LibBoard::Color::Black );
			//sprintf(buffer, "%.0f ms", statsThreadCrossedTime[i] * 1000.0);
			//board2 << Text(i * wUnit + wGap, - hIdle - hEmpty - hCrossed / 2 - 20, string(buffer), Fonts::CourierBold, 6, Color::Black );


			//Draw Empty in Red
			board2 << LibBoard::Rectangle(i * wUnit + wGap, -hIdle, wUnitBar, hEmpty, LibBoard::Color(0,0,0), LibBoard::Color(255, 0, 0), 0.01);
			sprintf(buffer, "%.0f ms/%u", statsThreadEmptyTime[i]*1000.0, statsThreadEmpty[i]);
			board2 << LibBoard::Text(i * wUnit + wGap, -hIdle - hEmpty / 2, string(buffer), Fonts::CourierBold, 6, LibBoard::Color::Black );
			//sprintf(buffer, "%.0f ms", statsThreadEmptyTime[i] * 1000.0);
			//board2 << Text(i * wUnit + wGap, - hIdle - hEmpty / 2 - 20, string(buffer), Fonts::CourierBold, 6, Color::Black );


			//Draw Idle in Gray
			board2 << LibBoard::Rectangle(i * wUnit + wGap, 0, wUnitBar, hIdle, LibBoard::Color(0,0,0), LibBoard::Color(128, 128, 128), 0.01);
			sprintf(buffer, "%.0f ms", tsIdle * 1000.0);
			board2 << LibBoard::Text(i * wUnit + wGap, -hIdle / 2, string(buffer), Fonts::CourierBold, 6, LibBoard::Color::Black );

			sprintf(buffer, "T%d", i+1);
			board2 << LibBoard::Text(i * wUnit + wGap, -HEIGHT + wGap, string(buffer), Fonts::CourierBold, 6, LibBoard::Color::White );
		}

		sprintf(buffer, "%s/graphs/CoreVertUsageXP%d.eps", chrFilePath, xpID);
		board2.saveEPS(buffer, WIDTH, HEIGHT);
		sprintf(buffer, "%s/graphs/CoreVertUsageXP%d.svg", chrFilePath, xpID);
		board2.saveSVG(buffer, WIDTH, HEIGHT);

	}

	return true;
}

