#include <map>
#include <assert.h>

#include "graphics/selectgl.h"
#include "graphics/ArcBallCamera.h"
#include "graphics/GLFuncs.h"
#include "graphics/AppScreen.h"
#include "graphics/cl_vector.h"

#include "tbb/task_scheduler_init.h"
#include "tbb/tick_count.h"
#include "sqlite/sqlite3.h"

#include "base/DebugUtils.h"
#include "base/Logger.h"
#include "base/SIMDVecN.h"
#include "base/ProcessorInfo.h"
#include "implicit/OclPolygonizer.h"
#include "implicit/PolyMemManager.h"
#include "implicit/ReadSceneModel.h"

#include "db.h"
#include "plot.h"

using namespace PS;
using namespace PS::CL;


#define WINDOW_SIZE_WIDTH 1024
#define WINDOW_SIZE_HEIGHT 768
#define PERS_FOV 60.0f
#define PERS_ZNEAR 0.1f
#define PERS_ZFAR 1000.0f
#define ORTHO_RIGHT	1.0f
#define ORTHO_TOP	1.0f

/*!
 * Application Settings. Drawing Settings and Performance configuration.
 */
struct APPSETTINGS{
	bool bPrimBoxes;
	bool bOpBoxes;
	bool bModelBox;
	bool bMPUBoxes;
	bool bDrawNormals;
	bool bDrawWireFrame;
	bool bReady;
	bool bInteractive;
	bool bPanCamera;
	bool bRenderGPU;

	//cellSize
	float cellsizeStart;
	float cellsizeEnd;
	float cellsizeStep;
	float cellsize;

	//gridDim
	int mpuDimStart;
	int mpuDimEnd;
	int mpuDimStep;
	MPUDim mpuDim;

	//threads
	int ctThreadCountStart;
	int ctThreadCountEnd;
	int ctThreads;
	U8  ctSIMDLength;

	bool bRoot;
	bool bPrintCharts;
	bool bExportObj;
	bool bDrawUtilizationGraph;
	bool bDrawUtilizationCellProcessInfo;
	float UtilizationWidthPercentFromEnd;

	int idxPrintPlotStart;
	int idxPrintPlotEnd;
	U32 attempts;

};


//Vertex Shader Code
const char * g_lpVertexShaderCode = 
	"varying vec3 N;"
	"varying vec3 V; "
	"void main(void) {"
	"gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;"
	"gl_FrontColor = gl_Color;"
	"N = normalize(gl_NormalMatrix * gl_Normal);"
	"V = vec3(gl_ModelViewMatrix * gl_Vertex); }";

//Fragment Shader Code
const char* g_lpFragShaderCode =
	"varying vec3 N;"
	"varying vec3 V;"
	"void main(void) {"
	"vec3 L = normalize(gl_LightSource[0].position.xyz - V);"
	"vec3 E = normalize(-V);"
	"vec3 R = normalize(-reflect(L, N));"
	"vec4 Iamb = 0.5 * gl_LightSource[0].ambient * gl_Color;"
	"vec4 Idif = (gl_LightSource[0].diffuse * gl_Color) * max(dot(N,L), 0.0);"
	"vec4 Ispec = (gl_LightSource[0].specular * (vec4(0.8, 0.8, 0.8, 0.8) + 0.2 * gl_Color)) * pow(max(dot(R, E), 0.0), 32.0);"
	"gl_FragColor = gl_FrontLightModelProduct.sceneColor + Iamb + Idif + Ispec;	}";

//Global Vars
ProcessorInfo g_cpuInfo;
PS::ArcBallCamera	g_arcBallCam;
APPSETTINGS g_appSettings;
GLuint g_uiShader;

//All Polygonizer data
SimdPoly      g_cpuPoly;
PS::SKETCH::GPUPoly* g_lpGPUPoly = NULL;

GLuint 		  g_textureId;
vec2f		  g_pan;

tbb::task_scheduler_init* g_taskSchedular = NULL;

//Graphics Functions
void DrawPolygonizerOutput(const PolyMPUs& polyMPUs);

//Performs an optimized SIMD polygonization and rendering
bool Run_CPUPoly(const AnsiStr& strModelFP);

//Runs OpenCL GPU polygonizer
bool Run_GPUPoly(const AnsiStr& strModelFP);

//GLUT CallBacks
void Close();
void Draw();
void Resize(int w, int h);
void Keyboard(int key, int x, int y);
void MousePress(int button, int state, int x, int y);
void MouseMove(int x, int y);
void MousePassiveMove(int x, int y);

//Charts
void ProduceUsageChartAsTexture(const char* chrDBPath,
							    const PolyMPUs& polyMPUs,
							    MPUSTATS* lpMPUStats,
							    tick_count& t0, tick_count& t1,
							    int xpID, int ctThreads,
							    float offsetPercentFromEnd = 1.0f);

//Logging to sqlite DB
//Insert record in perflog table
bool sqlite_InsertLogRecord(const char* chrDBPath, const char* lpStrModelName,
							MPUSTATS* lpStats,
							tick_count& t0, tick_count& t1,
							U8 ctSIMDLENGTH, U32 ctThreads,
							U32 szWorkMem, U32 szTotalMem, U32 szLLC);

//Insert record in utilization table
bool sqlite_InsertUtilRecord(const char* chrDBPath,
							 int xpID, int ctThreads,
							 U32* arrCrossed, U32* arrNonCrossed,
							 double* arrCrossedTime, double* arrNonCrossedTime, double* arrTTF,
							 int idxFastestCore, int idxSlowestCore);

//////////////////////////////////////////////////////////////////////////
void DrawPolygonizerOutput(const PolyMPUs& polyMPUs)
{
	if(polyMPUs.countWorkUnits() == 0) return;

	if(g_appSettings.bDrawWireFrame)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	const U32 mpuMeshVertexStrideF = polyMPUs.globalMesh.mpuMeshVertexStrideF;
	const U32 mpuMeshTriangleStrideU32 = polyMPUs.globalMesh.mpuMeshTriangleStrideU32;

	U32 ctWorkUnit = polyMPUs.countWorkUnits();
	for (U32 i=0; i<ctWorkUnit; i++)
	{
		if(polyMPUs.lpMPUs[i].ctTriangles > 0)
		{
			U32 szVertexPart = i * mpuMeshVertexStrideF;
			U32 szTrianglePart = i * mpuMeshTriangleStrideU32;

			//Color is RGB
			glColorPointer(3, GL_FLOAT, 0, &polyMPUs.globalMesh.vColor[szVertexPart]);
			glEnableClientState(GL_COLOR_ARRAY);

			//Normals
			glNormalPointer(GL_FLOAT, 0, &polyMPUs.globalMesh.vNorm[szVertexPart]);
			glEnableClientState(GL_NORMAL_ARRAY);

			//Vertex Unit is 3
			glVertexPointer(3, GL_FLOAT, 0, &polyMPUs.globalMesh.vPos[szVertexPart]);
			glEnableClientState(GL_VERTEX_ARRAY);

			glDrawElements(GL_TRIANGLES, (GLsizei)polyMPUs.lpMPUs[i].ctTriangles * 3, GL_UNSIGNED_SHORT, &polyMPUs.globalMesh.vTriangles[szTrianglePart]);

			glDisableClientState(GL_COLOR_ARRAY);
			glDisableClientState(GL_NORMAL_ARRAY);
			glDisableClientState(GL_VERTEX_ARRAY);

			if(g_appSettings.bDrawNormals)
			{
				glPushAttrib(GL_ALL_ATTRIB_BITS);
				glColor3f(0.0f, 0.0f, 1.0f);

				for(U32 j=0; j<polyMPUs.lpMPUs[i].ctVertices; j++)
				{
					vec3f v = vec3f(polyMPUs.globalMesh.vPos[szVertexPart + j*3],
									  polyMPUs.globalMesh.vPos[szVertexPart + j*3 + 1],
									  polyMPUs.globalMesh.vPos[szVertexPart + j*3 + 2]);
					vec3f n = vec3f(polyMPUs.globalMesh.vNorm[szVertexPart + j*3],
									  polyMPUs.globalMesh.vNorm[szVertexPart + j*3 + 1],
									  polyMPUs.globalMesh.vNorm[szVertexPart + j*3 + 2]);

					vec3f e = vec3f::add(v, vec3f::mul(0.5f, n));

					glBegin(GL_LINES);
					glVertex3f(v.x, v.y, v.z);
					glVertex3f(e.x, e.y, e.z);
					glEnd();
				}
				glPopAttrib();
			}
		}
	}
}

void Draw()
{
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	glLoadIdentity();

	//Print the plot
	if(g_appSettings.bDrawUtilizationGraph)
	{
		glUseProgram(0);
		glDisable(GL_LIGHTING);
		glPushAttrib(GL_ALL_ATTRIB_BITS);
			glBindTexture(GL_TEXTURE_2D, g_textureId);
			glBegin(GL_QUADS);
				glTexCoord2f(0.0, 0.0f);
				glVertex2f(0.0, 0.0f);

				glTexCoord2f(1.0, 0.0f);
				glVertex2f(1.0, 0.0f);

				glTexCoord2f(1.0, 1.0f);
				glVertex2f(1.0, 1.0f);

				glTexCoord2f(0.0, 1.0f);
				glVertex2f(0.0, 1.0f);
			glEnd();
			glBindTexture(GL_TEXTURE_2D, 0);
		glPopAttrib();
		glEnable(GL_LIGHTING);
	}
	else
	{
		vec3f p = g_arcBallCam.getPos();
		vec3f c = g_arcBallCam.getCenter();


		glTranslatef(g_pan.x, g_pan.y, 0.0f);
		gluLookAt(p.x, p.y, p.z, c.x, c.y, c.z, 0.0f, 1.0f, 0.0f);


		if(g_appSettings.bInteractive)
		{
			bool bScalar = (g_appSettings.ctSIMDLength == 1);
			g_appSettings.bReady = (g_cpuPoly.prepareBBoxes(g_appSettings.cellsize, g_appSettings.mpuDim.dim()) == RET_SUCCESS);
			g_appSettings.bReady &= (g_cpuPoly.polygonize(g_appSettings.cellsize, bScalar, NULL) == RET_SUCCESS);
			g_appSettings.bReady &= g_cpuPoly.extractSingleMeshObject();
		}

		//Draw Mesh
		glUseProgram(g_uiShader);

		if(g_appSettings.bRenderGPU) {
			if(g_lpGPUPoly) {
				g_lpGPUPoly->setWireFrameMode(g_appSettings.bDrawWireFrame);
				g_lpGPUPoly->draw();
			}
		}
		else
		{
			g_cpuPoly.setWireFrameMode(g_appSettings.bDrawWireFrame);
			g_cpuPoly.draw();
			if(g_appSettings.bDrawNormals)
				g_cpuPoly.drawMeshNormals();
		}

		glUseProgram(0);

		//Draw Spatial Structures
		if(g_appSettings.bReady)
		{
			//Using solid colors only for boxes
			glDisable(GL_LIGHTING);

			if(g_appSettings.bModelBox)
			{
				DrawAABB(g_cpuPoly.getBlobPrims()->bboxLo, g_cpuPoly.getBlobPrims()->bboxHi, vec3f(1.0f, 0.0f, 0.0f));
			}

			if(g_appSettings.bOpBoxes)
			{
				for(U32 i=0; i<g_cpuPoly.getBlobOps()->count; i++)
				{
					vec3f lo = vec3f(g_cpuPoly.getBlobOps()->bboxLoX[i], g_cpuPoly.getBlobOps()->bboxLoY[i], g_cpuPoly.getBlobOps()->bboxLoZ[i]);
					vec3f hi = vec3f(g_cpuPoly.getBlobOps()->bboxHiX[i], g_cpuPoly.getBlobOps()->bboxHiY[i], g_cpuPoly.getBlobOps()->bboxHiZ[i]);

					DrawAABB(lo, hi, vec3f(1.0f, 1.0f, 0.0));
				}
			}

			if(g_appSettings.bPrimBoxes)
			{
				for(U32 i=0; i<g_cpuPoly.getBlobPrims()->count; i++)
				{
					vec3f lo = vec3f(g_cpuPoly.getBlobPrims()->bboxLoX[i], g_cpuPoly.getBlobPrims()->bboxLoY[i], g_cpuPoly.getBlobPrims()->bboxLoZ[i]);
					vec3f hi = vec3f(g_cpuPoly.getBlobPrims()->bboxHiX[i], g_cpuPoly.getBlobPrims()->bboxHiY[i], g_cpuPoly.getBlobPrims()->bboxHiZ[i]);

					DrawAABB(lo, hi, vec3f(0.0f, 1.0f, 0.0));
				}
			}

			if(g_appSettings.bMPUBoxes)
			{
				float cpm = (float)(g_appSettings.mpuDim.dim() - 1);
				for(U32 i=0; i<g_cpuPoly.getMPUs()->countWorkUnits(); i++)
				{
					vec3f lo = g_cpuPoly.getMPUs()->lpMPUs[i].bboxLo;
					vec3f hi = vec3f::add(lo, vec3f::mul(g_appSettings.cellsize, vec3f(cpm, cpm, cpm)));

					DrawAABB(lo, hi, vec3f(0.0f, 0.0f, 1.0));
				}
			}

			glEnable(GL_LIGHTING);
		}
	}

	glutSwapBuffers();	
}

void Keyboard(int key, int x, int y)
{
	if(key == GLUT_KEY_F1)
	{
		g_appSettings.bModelBox = !g_appSettings.bModelBox;
		printf("Draw model box set = %d \n", g_appSettings.bModelBox);
	}
	else if(key == GLUT_KEY_F2)
	{
		g_appSettings.bOpBoxes = !g_appSettings.bOpBoxes;
		printf("Draw operator boxes set = %d \n", g_appSettings.bOpBoxes);
	}
	else if(key == GLUT_KEY_F3)
	{
		g_appSettings.bPrimBoxes = !g_appSettings.bPrimBoxes;
		printf("Draw primitive boxes set = %d \n", g_appSettings.bPrimBoxes);
	}
	else if(key == GLUT_KEY_F4)
	{
		g_appSettings.bMPUBoxes = !g_appSettings.bMPUBoxes;
		printf("Draw MPU boxes set = %d \n", g_appSettings.bMPUBoxes);
	}
	else if(key == GLUT_KEY_F5)
	{
		g_appSettings.bDrawWireFrame = !g_appSettings.bDrawWireFrame;
		printf("Draw Wireframe set = %d \n", g_appSettings.bDrawWireFrame);
	}

	else if(key == GLUT_KEY_F6)
	{
		g_appSettings.bDrawNormals = !g_appSettings.bDrawNormals;
		printf("Draw Normals set = %d \n", g_appSettings.bDrawNormals);
	}
	else if(key == GLUT_KEY_F7)
	{
		g_appSettings.bDrawUtilizationGraph = !g_appSettings.bDrawUtilizationGraph;
		printf("Draw Utilization Graph set = %d \n", g_appSettings.bDrawUtilizationGraph);
		printf("Util width set = %.2f \n", g_appSettings.UtilizationWidthPercentFromEnd);
		Resize(WINDOW_SIZE_WIDTH, WINDOW_SIZE_HEIGHT);
	}
	else if(key == GLUT_KEY_F8)
	{
		g_appSettings.bPanCamera = !g_appSettings.bPanCamera;
		printf("Pan Camera set = %d \n", g_appSettings.bPanCamera);
	}
	else if(key == GLUT_KEY_F9)
	{
		g_appSettings.UtilizationWidthPercentFromEnd -= 0.1f;
		Clamp<float>(g_appSettings.UtilizationWidthPercentFromEnd, 0.0f, 1.0f);
		printf("Util width set = %.2f \n", g_appSettings.UtilizationWidthPercentFromEnd);
	}
	else if(key == GLUT_KEY_F10)
	{
		g_appSettings.UtilizationWidthPercentFromEnd += 0.1f;
		Clamp<float>(g_appSettings.UtilizationWidthPercentFromEnd, 0.0f, 1.0f);
		printf("Util width set = %.2f \n", g_appSettings.UtilizationWidthPercentFromEnd);
	}
	else if(key == GLUT_KEY_F11)
	{
		printf("Exiting...\n");
		glutLeaveMainLoop();
		//Close();
	}


	glutPostRedisplay();
}

void MousePress(int button, int state, int x, int y)
{
	// Wheel reports as button 3(scroll up) and button 4(scroll down)
	if ((button == 3) || (button == 4)) {
		// Each wheel event reports like a button click, GLUT_DOWN then GLUT_UP
		if (state == GLUT_UP)
			return; // Disregard redundant GLUT_UP events

		if (button == 3)
			g_arcBallCam.setZoom(g_arcBallCam.getZoom() - 0.5);
		else
			g_arcBallCam.setZoom(g_arcBallCam.getZoom() + 0.5);
	}

	g_arcBallCam.mousePress(button, state, x, y);

	glutPostRedisplay();
}

void MouseMove(int x, int y)
{
	if(g_appSettings.bPanCamera)
	{
		g_pan.x += (x - g_arcBallCam.getLastPos().x) * 0.03f;
		g_pan.y += (g_arcBallCam.getLastPos().y - y) * 0.03f;
		g_arcBallCam.setLastPos(vec2i(x, y));
	}
	else
		g_arcBallCam.mouseMove(x, y);
	glutPostRedisplay();
}

void MousePassiveMove(int x, int y) {

}

void MouseWheel(int button, int dir, int x, int y)
{
	g_arcBallCam.mouseWheel(button, dir, x, y);
	glutPostRedisplay();
}

void Resize(int w, int h)
{
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if(g_appSettings.bDrawUtilizationGraph)
		gluOrtho2D(0.0f, ORTHO_RIGHT, 0.0f, ORTHO_TOP);
	else
		gluPerspective(PERS_FOV, (float)w / (float)h, PERS_ZNEAR, PERS_ZFAR);
	
	glMatrixMode(GL_MODELVIEW);
}


bool sqlite_InsertLogRecord(const char* chrDBPath, const char* lpStrModelName,
							MPUSTATS* lpStats,
							tick_count& t0, tick_count& t1,
							U8 ctSIMDLENGTH, U32 ctThreads,
							U32 szWorkMem, U32 szTotalMem, U32 szLLC)
{
	sqlite3* db;
	char* lpSqlError = 0;

	int rc = sqlite3_open(chrDBPath, &db);
	if(rc)
	{
		fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
		sqlite3_close(db);
	}

	//////////////////////////////////////////////////////////
	//Model Name
	const int MODEL_STRING_LENGTH = 30;
	char chrModelName[MODEL_STRING_LENGTH];
	{
		strncpy(chrModelName, lpStrModelName, MODEL_STRING_LENGTH);
	}

	//Get Time
	const int TIME_STRING_LENGTH = 20;
	char chrTimeStamp[TIME_STRING_LENGTH];
	{
		time_t rawtime;
		struct tm* currentTime;
		time(&rawtime);
		currentTime = localtime(&rawtime);
		strftime(chrTimeStamp, TIME_STRING_LENGTH, "%Y-%m-%d %H:%M:%S", currentTime);
	}

	U32 ctMPUIntersected = 0;
	U32 ctMeshFaces = 0;
	U32 ctMeshVertices = 0;
	U32 ctFieldEvals = 0;
	U32 ctMaxFieldEvals = 0;
	U32 ctWorkUnits = g_cpuPoly.getMPUs()->countWorkUnits();
	double msPolyAvgFieldEvals = 0.0;
	double msPolyAvgTriangulate = 0.0;
	double msPolyTotal = 0.0;

	for(U32 i=0; i<ctWorkUnits; i++)
	{
		U32 ctCurFieldEvals =  g_cpuPoly.getMPUs()->lpMPUs[i].ctFieldEvals;
		if(ctCurFieldEvals > ctMaxFieldEvals)
			ctMaxFieldEvals = ctCurFieldEvals;

		ctFieldEvals += ctCurFieldEvals;
		if(g_cpuPoly.getMPUs()->lpMPUs[i].ctTriangles > 0)
		{
			ctMPUIntersected++;
			ctMeshFaces += g_cpuPoly.getMPUs()->lpMPUs[i].ctTriangles;
			ctMeshVertices += g_cpuPoly.getMPUs()->lpMPUs[i].ctVertices;
			if(lpStats)
			{
				msPolyAvgTriangulate += (lpStats[i].tickEnd - lpStats[i].tickFieldEvals).seconds() * 1000.0;
			}
		}

		if(lpStats)
		{
			msPolyAvgFieldEvals += (lpStats[i].tickFieldEvals - lpStats[i].tickStart).seconds() * 1000.0;
		}
	}

	{
		msPolyAvgFieldEvals /= (double)ctThreads;
		msPolyAvgTriangulate /= (double)ctThreads;
		msPolyTotal = (t1 - t0).seconds() * 1000.0;
		msPolyTotal /= (double)g_appSettings.attempts;
		double fps = (msPolyTotal != 0.0)? 1000.0 / msPolyTotal : 1000.0;
		vec3i dim = g_cpuPoly.getMPUs()->getWorkDim();
		printf("Processed %d MPUs [%d, %d, %d] TIME=%.2f, FPS=%.2f. Avg [FE=%.2f, Tri=%.2f]\n",
				(int)ctWorkUnits, dim.x, dim.y, dim.z, msPolyTotal, fps, msPolyAvgFieldEvals, msPolyAvgTriangulate);
	}

	U32 fvept = (ctMeshFaces > 0)? ctFieldEvals / ctMeshFaces : ctFieldEvals;

	//////////////////////////////////////////////////////////
	const char *strSQL = "INSERT INTO tblPerfLog (xpModelName, xpTime, ctPrims, ctOps, mpuDim, cellSize, ctGroupsTotal, ctMPUTotal, ctMPUIntersected, ctTotalFieldEvals, "
						"ctLatestMPUFieldEvals, ctFVEPT, ctMeshFaces, ctMeshVertices, msPolyTotal, msPolyFieldEvals, msPolyTriangulate, "
						"ctThreads, ctSIMDLength, szWorkItemMem, szTotalMemUsage, szLastLevelCache) values"
						"(@xp_model_name, @xp_time, @ct_prims, @ct_ops, @mpu_dim, @cell_size, @ct_groups_total, @ct_mpu_total, @ct_mpu_intersected, @ct_total_fieldevals, "
						"@ct_lastestmpu_fieldevals, @ct_fvept, @ct_mesh_faces, @ct_mesh_vertices, @time_total, @time_fieldevals, @time_triangulate, "
						"@ct_threads, @ct_simd_length, @sz_work_item_mem, @sz_total_mem_usage, @sz_last_level_cache);";

						 //"ctThreads smallint, ctSIMDLength smallint, szWorkItemMem int, szTotalMemUsage int, szLastLevelCache int);";
	sqlite3_stmt* statement;
	sqlite3_prepare_v2(db, strSQL, -1, &statement, NULL);

	//Bind All Parameters for Insert Statement
	//int ctParams = sqlite3_bind_parameter_count(statement);

	int idxParam = sqlite3_bind_parameter_index(statement, "@xp_model_name");
	sqlite3_bind_text(statement, idxParam, chrModelName, -1, SQLITE_TRANSIENT);

	idxParam = sqlite3_bind_parameter_index(statement, "@xp_time");
	sqlite3_bind_text(statement, idxParam, chrTimeStamp, -1, SQLITE_TRANSIENT);

	idxParam = sqlite3_bind_parameter_index(statement, "@ct_prims");
	sqlite3_bind_int(statement, idxParam, g_cpuPoly.getBlobPrims()->count);

	idxParam = sqlite3_bind_parameter_index(statement, "@ct_ops");
	sqlite3_bind_int(statement, idxParam, g_cpuPoly.getBlobOps()->count);

	idxParam = sqlite3_bind_parameter_index(statement, "@mpu_dim");
	sqlite3_bind_int(statement, idxParam, g_appSettings.mpuDim.dim());

	idxParam = sqlite3_bind_parameter_index(statement, "@cell_size");
	sqlite3_bind_double(statement, idxParam, g_appSettings.cellsize);

	idxParam = sqlite3_bind_parameter_index(statement, "@ct_groups_total");
	sqlite3_bind_int(statement, idxParam, 1);

	idxParam = sqlite3_bind_parameter_index(statement, "@ct_mpu_total");
	sqlite3_bind_int(statement, idxParam, g_cpuPoly.getMPUs()->countWorkUnits());

	idxParam = sqlite3_bind_parameter_index(statement, "@ct_mpu_intersected");
	sqlite3_bind_int(statement, idxParam, ctMPUIntersected);

	idxParam = sqlite3_bind_parameter_index(statement, "@ct_total_fieldevals");
	sqlite3_bind_int(statement, idxParam, ctFieldEvals);

	//Maximum number of field-evals per MPU
	idxParam = sqlite3_bind_parameter_index(statement, "@ct_lastestmpu_fieldevals");
	sqlite3_bind_int(statement, idxParam, ctMaxFieldEvals);

	idxParam = sqlite3_bind_parameter_index(statement, "@ct_fvept");
	sqlite3_bind_int(statement, idxParam, fvept);

	idxParam = sqlite3_bind_parameter_index(statement, "@ct_mesh_faces");
	sqlite3_bind_int(statement, idxParam, ctMeshFaces);

	idxParam = sqlite3_bind_parameter_index(statement, "@ct_mesh_vertices");
	sqlite3_bind_int(statement, idxParam, ctMeshVertices);

	idxParam = sqlite3_bind_parameter_index(statement, "@time_total");
	sqlite3_bind_double(statement, idxParam, msPolyTotal);
	//?
	idxParam = sqlite3_bind_parameter_index(statement, "@time_fieldevals");
	sqlite3_bind_double(statement, idxParam, msPolyAvgFieldEvals);

	idxParam = sqlite3_bind_parameter_index(statement, "@time_triangulate");
	sqlite3_bind_double(statement, idxParam, msPolyAvgTriangulate);

	//Hardware Log
	idxParam = sqlite3_bind_parameter_index(statement, "@ct_threads");
	sqlite3_bind_int(statement, idxParam, ctThreads);

	idxParam = sqlite3_bind_parameter_index(statement, "@ct_simd_length");
	sqlite3_bind_int(statement, idxParam, ctSIMDLENGTH);

	idxParam = sqlite3_bind_parameter_index(statement, "@sz_work_item_mem");
	sqlite3_bind_int(statement, idxParam, szWorkMem);

	idxParam = sqlite3_bind_parameter_index(statement, "@sz_total_mem_usage");
	sqlite3_bind_int(statement, idxParam, szTotalMem);

	idxParam = sqlite3_bind_parameter_index(statement, "@sz_last_level_cache");
	sqlite3_bind_int(statement, idxParam, szLLC);


	rc = sqlite3_step(statement);
	if(rc != SQLITE_DONE)
	{
		fprintf(stderr, "SQL error: %s\n", sqlite3_errmsg(db));
		sqlite3_free(lpSqlError);
	}

	//Free Statement
	sqlite3_finalize(statement);

	//Close DB
	sqlite3_close(db);

	return true;
}


bool sqlite_InsertUtilRecord(const char* chrDBPath, int xpID, int ctThreads, U32* arrCrossed, U32* arrNonCrossed,
							 double* arrCrossedTime, double* arrNonCrossedTime, double* arrLatestTime,
							 int idxFastestCore, int idxSlowestCore)
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

	//////////////////////////////////////////////////////////
	//Model Name
	char buffer[2048];
	string strSQL = "INSERT INTO tblUtilization (xpID, ctCores, ";
	for(int i=1; i<=ctThreads; i++)
	{
		sprintf(buffer, "mpuCrossed%d, mpuNonCrossed%d, mpuCrossedTime%d, mpuNonCrossedTime%d, latestFinish%d, ", i, i, i, i, i);
		strSQL += string(buffer);
	}
	strSQL += "fastest, slowest) values (@xp_id, @ct_cores, ";
	for(int i=1; i<=ctThreads; i++)
	{
		sprintf(buffer, "@mpu_crossed%d, @mpu_noncrossed%d, @mpu_crossedtime%d, @mpu_noncrossedtime%d, @latest_finish%d, ", i, i, i, i, i);
		strSQL += string(buffer);
	}
	strSQL += "@fastest, @slowest);";


	sqlite3_stmt* statement;
	sqlite3_prepare_v2(db, strSQL.c_str(), -1, &statement, NULL);

	//Bind All Parameters for Insert Statement
	//int ctParams = sqlite3_bind_parameter_count(statement);

	int idxParam = sqlite3_bind_parameter_index(statement, "@xp_id");
	sqlite3_bind_int(statement, idxParam, xpID);

	idxParam = sqlite3_bind_parameter_index(statement, "@ct_cores");
	sqlite3_bind_int(statement, idxParam, ctThreads);

	idxParam = sqlite3_bind_parameter_index(statement, "@fastest");
	sqlite3_bind_int(statement, idxParam, idxFastestCore + 1);

	idxParam = sqlite3_bind_parameter_index(statement, "@slowest");
	sqlite3_bind_int(statement, idxParam, idxSlowestCore + 1);


	for(int i=1; i<=ctThreads; i++)
	{
		sprintf(buffer, "@mpu_crossed%d", i);
		idxParam = sqlite3_bind_parameter_index(statement, buffer);
		sqlite3_bind_int(statement, idxParam, arrCrossed[i-1]);

		sprintf(buffer, "@mpu_noncrossed%d", i);
		idxParam = sqlite3_bind_parameter_index(statement, buffer);
		sqlite3_bind_int(statement, idxParam, arrNonCrossed[i-1]);

		sprintf(buffer, "@mpu_crossedtime%d", i);
		idxParam = sqlite3_bind_parameter_index(statement, buffer);
		sqlite3_bind_double(statement, idxParam, arrCrossedTime[i-1] * 1000.0);

		sprintf(buffer, "@mpu_noncrossedtime%d", i);
		idxParam = sqlite3_bind_parameter_index(statement, buffer);
		sqlite3_bind_double(statement, idxParam, arrNonCrossedTime[i-1] * 1000.0);

		sprintf(buffer, "@latest_finish%d", i);
		idxParam = sqlite3_bind_parameter_index(statement, buffer);
		sqlite3_bind_double(statement, idxParam, arrLatestTime[i-1] * 1000.0);
	}

	rc = sqlite3_step(statement);
	if(rc != SQLITE_DONE)
	{
		fprintf(stderr, "SQL error: %s\n", sqlite3_errmsg(db));
		sqlite3_free(lpSqlError);
	}

	//Free Statement
	sqlite3_finalize(statement);

	//Close DB
	sqlite3_close(db);

	return true;
}



/*!
 * Main Executable function
 */
int main(int argc, char* argv[])
{
	printf("High Performance Implicit Renderer - Pourya Shirazian [Use -h for usage.]\n");
	printf("**********************************************************************************\n");
	//Detect CPU Info
	if(g_cpuInfo.bOSSupportAVX)
		printf("OS supports AVX\n");
	else
		printf("OS DOESNOT support AVX\n");

	if(g_cpuInfo.bSupportAVX)
		printf("AVX supported.\n");
	else
		printf("AVX is not supported on this machine.\n");
	if(g_cpuInfo.bSupportAVX2)
		printf("AVX2 supported.\n");
	else
		printf("AVX2 is not supported on this machine.\n");

	printf("Machine: CoresCount:%u, SIMD FLOATS:%u\n", g_cpuInfo.ctCores, g_cpuInfo.simd_float_lines);
	printf("CacheLine:%u [B]\n", g_cpuInfo.cache_line_size);
	for(int i=0; i<g_cpuInfo.ctCacheInfo; i++)
	{
		if(g_cpuInfo.cache_types[i] != ProcessorInfo::ctNone)
		{
			printf("\tLevel:%d, Type:%s, Size:%u [KB]\n",
					g_cpuInfo.cache_levels[i],
					ProcessorInfo::GetCacheTypeString(g_cpuInfo.cache_types[i]),
					g_cpuInfo.cache_sizes[i] / 1024);
		}
	}
	printf("**********************************************************************************\n");
	AnsiStr strModelFP;
	memset(&g_appSettings, 0, sizeof(APPSETTINGS));
	g_appSettings.attempts = 1;
	g_appSettings.cellsize = DEFAULT_CELL_SIZE;
	g_appSettings.cellsizeStart = DEFAULT_CELL_SIZE;
	g_appSettings.cellsizeEnd = DEFAULT_CELL_SIZE;
	g_appSettings.cellsizeStep  = DEFAULT_CELL_SIZE;

	g_appSettings.mpuDim.set(DEFAULT_MPU_DIM);
	g_appSettings.mpuDimStart = DEFAULT_MPU_DIM;
	g_appSettings.mpuDimEnd = DEFAULT_MPU_DIM;
	g_appSettings.mpuDimStep = DEFAULT_MPU_DIM;

	g_appSettings.ctSIMDLength = PS_SIMD_FLEN;
	g_appSettings.ctThreads = g_cpuInfo.ctCores;
	g_appSettings.ctThreads = g_cpuInfo.ctCores;
	g_appSettings.ctThreadCountStart = g_cpuInfo.ctCores;
	g_appSettings.ctThreadCountEnd   = g_cpuInfo.ctCores;
	g_appSettings.UtilizationWidthPercentFromEnd = 1.0f;
	g_appSettings.bRoot = false;
	g_appSettings.bRenderGPU = false;
	g_appSettings.bPrintCharts = false;

	for(int i=0; i<argc; i++)
	{
		std::string strArg = argv[i];

		if(std::strcmp(strArg.c_str(), "-a") == 0)
		{
			g_appSettings.attempts = atoi(argv[i+1]);
		}


		if(std::strcmp(strArg.c_str(), "-g") == 0)
		{
			g_appSettings.bRenderGPU = true;
		}

		if(std::strcmp(strArg.c_str(), "-u") == 0)
		{
			g_appSettings.UtilizationWidthPercentFromEnd = static_cast<float>(atof(argv[i+1]));
		}

		if(std::strcmp(strArg.c_str(), "-d") == 0)
		{
			g_appSettings.bDrawUtilizationCellProcessInfo = true;
		}

		if(std::strcmp(strArg.c_str(), "-f") == 0)
		{
			strModelFP = AnsiStr(argv[i+1]);
		}

		if(std::strcmp(strArg.c_str(), "-c") == 0)
		{
			g_appSettings.cellsize = static_cast<float>(atof(argv[i+1]));
			g_appSettings.cellsizeStart = g_appSettings.cellsize;
			g_appSettings.cellsizeEnd = g_appSettings.cellsize;
		}
		else if(std::strcmp(strArg.c_str(), "-cc") == 0)
		{
			g_appSettings.cellsizeStart = static_cast<float>(atof(argv[i+1]));
			g_appSettings.cellsizeEnd   = static_cast<float>(atof(argv[i+2]));
			g_appSettings.cellsizeStep  = static_cast<float>(atof(argv[i+3]));
			g_appSettings.cellsize		= g_appSettings.cellsizeStart;
		}

		if(std::strcmp(strArg.c_str(), "-t") == 0)
		{
			g_appSettings.ctThreads = atoi(argv[i+1]);
			g_appSettings.ctThreadCountStart = g_appSettings.ctThreads;
			g_appSettings.ctThreadCountEnd = g_appSettings.ctThreads;
		}
		else if(std::strcmp(strArg.c_str(), "-tt") == 0)
		{
			g_appSettings.ctThreadCountStart = atoi(argv[i+1]);
			g_appSettings.ctThreadCountEnd = atoi(argv[i+2]);
			g_appSettings.ctThreads = g_appSettings.ctThreadCountStart;
		}

		if (std::strcmp(strArg.c_str(), "-m") == 0) {
			U8 dim = atoi(argv[i + 1]);
			if (!MPUDim::isValid(dim)) {
				LogErrorArg1("MPU dimension (%u) is invalid!", dim);
				exit(1);
			}

			LogInfoArg1("Set MPU dimension to (%u)", dim);
			g_appSettings.mpuDim.set(dim);
			g_appSettings.mpuDimStart = dim;
			g_appSettings.mpuDimEnd = dim;
			g_appSettings.mpuDimStep = dim;
		}
		else if (std::strcmp(strArg.c_str(), "-mm") == 0)
		{
			U8 dim = atoi(argv[i + 1]);
			if (!MPUDim::isValid(dim)) {
				LogErrorArg1("MPU dimension (%u) is invalid!", dim);
				exit(1);
			}

			LogInfoArg1("Set MPU dimension to (%u)", dim);
			g_appSettings.mpuDim.set(dim);
			g_appSettings.mpuDimStart = atoi(argv[i + 1]);
			g_appSettings.mpuDimEnd = atoi(argv[i + 2]);
			g_appSettings.mpuDimStep = atoi(argv[i + 3]);
		}

		if(std::strcmp(strArg.c_str(), "-r") == 0)
		{
			g_appSettings.bRoot = true;
		}

		if(std::strcmp(strArg.c_str(), "-p") == 0)
		{
			g_appSettings.bPrintCharts = true;
			g_appSettings.idxPrintPlotStart = atoi(argv[i+1]);
			g_appSettings.idxPrintPlotEnd = atoi(argv[i+2]);
		}

		if(std::strcmp(strArg.c_str(), "-o") == 0)
		{
			g_appSettings.bPrintCharts = true;
			g_appSettings.idxPrintPlotStart = -1;
			g_appSettings.idxPrintPlotEnd = -1;
		}

		if(std::strcmp(strArg.c_str(), "-e") == 0)
		{
			g_appSettings.bExportObj = true;
		}

		if(std::strcmp(strArg.c_str(), "-i") == 0)
		{
			g_appSettings.bInteractive = true;
		}

		if(std::strcmp(strArg.c_str(), "-s") == 0)
		{
			g_appSettings.ctSIMDLength = 1;
		}



		if(std::strcmp(strArg.c_str(), "-h") == 0)
		{
			printf("Usage: ParsipCmd -t [threads_count] -i -a [attempts] -c [cell_size] -f [model_file_path]\n");
			printf("-a \t [runs] Number of runs for stats.\n");
			printf("-c \t [cellsize] cell size parameter in float.\n");
			printf("-cc \t [Start - End - Step] cell size parameter interval in float.\n");
			printf("-t \t [threads count] Threads count.\n");
			printf("-tt \t [Start - End] Threads count interval.\n");
			printf("-m \t Sets the MPU grid dimension [default=8].\n");
			printf("-mm \t [Start - End - Step] MPU grid dim interval [default=8].\n");

			printf("-f \t [filepath] model file path in .scene format.\n");
			printf("-p \t [Start - End] Plot performance graph using db values in range.\n");
			printf("-u \t [Percentage] Utilization width offset from end.\n");
			printf("-e \t Export mesh as obj.\n");
			printf("-r \t Path is root.\n");
			printf("-s \t Run CPU polygonizer in scalar mode.\n");
			printf("-g \t Run GPU OpenCL polygonizer.\n");
			printf("-d \t Enables writing average cell processing info in utilization graph.\n");
			printf("-i \t Sets interactive mode. Each draw call will polygonize.\n");
			printf("-o \t Output MPU processing and Core utilization graphs in svg and eps.\n");

			return 0;
		}
	}

	{
		if(g_appSettings.cellsize < MIN_CELL_SIZE)
		{
			printf("Error: Invalid cellsize param.\n");
			return 0;
		}

		if(g_appSettings.ctThreadCountStart > g_appSettings.ctThreadCountEnd)
		{
			printf("Error: Invalid threads intervals.\n");
			return 0;
		}
	}


	//Init GLUT
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(WINDOW_SIZE_WIDTH, WINDOW_SIZE_HEIGHT);
	glutCreateWindow("ParsipSIMD - Cache Optimized SIMD Scalable Polygonizer for Skeletal Implicit Surfaces");
	glutDisplayFunc(Draw);
	glutReshapeFunc(Resize);
	glutMouseFunc(MousePress);
	glutMotionFunc(MouseMove);
	glutPassiveMotionFunc(MousePassiveMove);
	glutMouseWheelFunc(MouseWheel);

	glutSpecialFunc(Keyboard);
	glutCloseFunc(Close);
	//glutKeyboardFunc(Keyboard);

	//////////////////////////////////////////////////////////////////////////
	def_initgl();
	CompileShaderCode(g_lpVertexShaderCode, g_lpFragShaderCode, g_uiShader);

	{
		using namespace PS;
		using namespace PS::FILESTRINGUTILS;
		
		//For Debugging only
		if(g_appSettings.bRoot)
		{
			AnsiStr strFilePath =  ExtractOneLevelUp(ExtractFilePath(GetExePath()));
			strFilePath += AnsiStr(strModelFP.c_str());
			strModelFP = strFilePath;
		}
	}


	//CL test
	vector<int> arrVals;
	arrVals.resize(1024);
	for(int i=0; i < 1024; i++)
		arrVals[i] = i*i;
//	HostVector<int> host_vector(arrVals);
//	DeviceVector<int> device_vector(host_vector);
	DeviceVector<int> device_vector(arrVals);

	vector<int> vReadBack;
	device_vector.read(vReadBack);
	PS::DEBUGTOOLS::PrintArray(reinterpret_cast<U32*>( &vReadBack[0] ), 100);





	//Render CPU or GPU renderer
	bool bPolyRes = false;
	if(g_appSettings.bRenderGPU)
		bPolyRes = Run_GPUPoly(strModelFP);
	else
		bPolyRes = Run_CPUPoly(strModelFP);

	if(bPolyRes)
		glutMainLoop();

	return 0;
}

//GPU Polygonizer
bool Run_GPUPoly(const AnsiStr& strModelFP)
{
	printf("Start GPU OpenCL polygonizer: [cellsize = %.2f] [attempts = %u]\n", g_appSettings.cellsize, g_appSettings.attempts);
	LinearBlobTree blob;
	if(!blob.load(strModelFP))
	{
		printf("ERROR: Invalid input model!\n");
		return false;
	}

	printf("Using Input model %s.\n", strModelFP.c_str());

	if(g_lpGPUPoly == NULL)
		g_lpGPUPoly = new GPUPoly();
	g_lpGPUPoly->setBlob(blob);
	g_lpGPUPoly->setCellSize(g_appSettings.cellsize);
	g_lpGPUPoly->run();

	glutPostRedisplay();

	return true;
}

//GPU MultiCore Polygonizer with SIMD instructions
bool Run_CPUPoly(const AnsiStr& strModelFP)
{
	string strModelName;

	//Initialize Variables
	g_cpuPoly.allocate();
	if(!g_cpuPoly.readModel(strModelFP.c_str()))
	{
		printf("ERROR: Invalid input model!\n");
		return false;
	}

	printf("Using Input model %s.\n", strModelFP.c_str());
	g_cpuPoly.printModelInfo();
	strModelName = string(PS::FILESTRINGUTILS::ExtractFileName(AnsiStr(strModelFP.c_str())).cptr());
	//////////////////////////////////////////////////////////////////////////
	//Print all used memory
	U32 szWorkItemMem, szTotalMem, szLLC;
	{
		//Print total sizes
		U32 szPrims = sizeof(SOABlobPrims);
		U32 szOps 		= sizeof(SOABlobOps);
		U32 szNodeMatrices 	= sizeof(SOABlobNodeMatrices);

		U32 szMPU 		= sizeof(MPU);
		U32 szEdgeTable = EdgeTable::MemSizeInBytes(g_appSettings.mpuDim);
		U32 szFieldCache = g_appSettings.mpuDim.dim3() * sizeof(float);

		szWorkItemMem = szPrims + szNodeMatrices + szOps + szMPU + szEdgeTable + szFieldCache;
		szTotalMem = szWorkItemMem;

		szLLC = g_cpuInfo.getLastLevelSize();

		printf("Total sizes: BlobOps + BlobPrims + NodeMatrices + MPU + EDGETABLE + FIELDCACHE= %u, %u, %u, %u, %u, %u = %u bytes\n",
			   szOps, szPrims, szNodeMatrices, szMPU, szEdgeTable, szFieldCache, szWorkItemMem);
		if(szWorkItemMem < szLLC)
			printf("LLC can fit current work item. Free Space = %u [Bytes]\n", szLLC - szWorkItemMem);
		else
			printf("Warning! LLC overflow.\n");
	}

	//Get Log File Path
	string strLogFP;
	string strExePath;
	{
		using namespace PS::FILESTRINGUTILS;
		strExePath = (string)GetExePath().cptr();
		strLogFP = (string)(ExtractFilePath(GetExePath()) + AnsiStr("PerformanceLog.db")).cptr();
		sqlite_CreateTableIfNotExist(strLogFP.c_str(), g_cpuInfo.ctCores);
	}

	g_appSettings.attempts = MATHMAX(1, g_appSettings.attempts);
	printf("Start CPU polygonizer: [cellsize = %.2f] [attempts = %u]\n", g_appSettings.cellsize, g_appSettings.attempts);
	int ctCellSteps = (int)((g_appSettings.cellsizeEnd - g_appSettings.cellsizeStart)/ g_appSettings.cellsizeStep);

	//For Loop on number of threads
	for(int ctThreads = g_appSettings.ctThreadCountStart; ctThreads <= g_appSettings.ctThreadCountEnd; ctThreads++)
	{
		printf("*********************************************************************\n");
		g_appSettings.ctThreads = ctThreads;
		SAFE_DELETE(g_taskSchedular);
		g_taskSchedular = new tbb::task_scheduler_init(g_appSettings.ctThreads);
		printf("Parsip CMD - THREADS=%d, SIMD FLEN=%d\n", g_appSettings.ctThreads, g_appSettings.ctSIMDLength);

		//for: MPU dim
		for(int iMPUDim = g_appSettings.mpuDimStart;
				iMPUDim <= g_appSettings.mpuDimEnd;
				iMPUDim += g_appSettings.mpuDimStep) {

			//set MPU dim
			g_appSettings.mpuDim.set(iMPUDim);

			//for: cell size
			for(int iCellStep = 0; iCellStep <= ctCellSteps; iCellStep++)
			{
				g_appSettings.cellsize = g_appSettings.cellsizeStart + iCellStep * g_appSettings.cellsizeStep;
				printf("ParsipSimd - CellSize Param = %.2f, MPU DIM = %u \n", g_appSettings.cellsize, g_appSettings.mpuDim.dim());

				g_appSettings.bReady = g_cpuPoly.prepareBBoxes(g_appSettings.cellsize, g_appSettings.mpuDim.dim());
				vec3i dim = g_cpuPoly.getMPUs()->getWorkDim();
				vec3f lo = g_cpuPoly.getBlobPrims()->bboxLo;
				vec3f hi = g_cpuPoly.getBlobPrims()->bboxHi;
				printf("WorkGrid=[%d, %d, %d], BoxLo=[%.3f %.3f %.3f], BoxHi=[%.3f %.3f %.3f]\n",
						dim.x, dim.y, dim.z,
						lo.x, lo.y, lo.z,
						hi.x, hi.y, hi.z);

				//Create data structure to hold stats data
				MPUSTATS* lpMPUStats = new MPUSTATS[MATHMAX(1, g_cpuPoly.getMPUs()->countWorkUnits())];

				//POLYGONIZE
				bool bScalar = (g_appSettings.ctSIMDLength == 1);
				tbb::tick_count t0 = tbb::tick_count::now();
				for(U32 i=0; (i < g_appSettings.attempts)&&(g_appSettings.bReady); i++)
				{
					LogInfoArg2("Perform attempt %u of %u", i+1, g_appSettings.attempts);
					g_appSettings.bReady &= g_cpuPoly.polygonize(g_appSettings.cellsize, bScalar, lpMPUStats);
				}
				tbb::tick_count t1 = tbb::tick_count::now();

				PrintThreadResults(g_appSettings.attempts);
				//Insert record in log
				sqlite_InsertLogRecord(strLogFP.c_str(),
									   strModelName.c_str(),
									   lpMPUStats,
									   t0, t1,
									   g_appSettings.ctSIMDLength, g_appSettings.ctThreads,
									   szWorkItemMem, szTotalMem, szLLC);
				int xpID = sqlite_GetLastXPID(strLogFP.c_str());

				//Print and Produce Result Graphs
				//Thread Processing Results
				ProduceUsageChartAsTexture(strLogFP.c_str(),
										   *g_cpuPoly.getMPUs(), lpMPUStats,
										   t0, t1,
										   xpID, ctThreads,
										   g_appSettings.UtilizationWidthPercentFromEnd);

				if(g_appSettings.bPrintCharts)
					chart_CreateStackChart(xpID, *g_cpuPoly.getMPUs(), lpMPUStats, t0, t1, ctThreads);
				if(g_appSettings.bExportObj)
				{
					printf("Exporting mesh as obj file: xp%i.obj\n", xpID);
					char buffer[1024];
					sprintf(buffer, "%s_xp%d.obj", strExePath.c_str(), xpID);
					ExportMeshAsObj(buffer, *g_cpuPoly.getMPUs());
				}

				//delete stats data
				SAFE_DELETE(lpMPUStats);
				printf("*********************************************************************\n");
				glutPostRedisplay();
			}
		}
	}
	printf("*********************************************************************\n");
	//////////////////////////////////////////////////////////////////////////
	if(g_appSettings.bPrintCharts && (g_appSettings.idxPrintPlotStart >= 0))
	{
		printf("**********************************************************************************\n");
		printf("Preparing GNUPLOT Data in range [%d, %d] \n",
				g_appSettings.idxPrintPlotStart,
				g_appSettings.idxPrintPlotEnd);
		GNUPlotDriver* lpPlot = new GNUPlotDriver(strLogFP, g_appSettings.idxPrintPlotStart, g_appSettings.idxPrintPlotEnd);
		bool bres = lpPlot->createGraph();
		SAFE_DELETE(lpPlot);

		if(bres)
		{
			printf("Log.dat file created with format [ctThreads PolyTime] just run gnuplot on it.\n");
			printf("Example: plot \"./ParsipSIMDLinuxLog.dat\" using 1:2 title \"Performance\" with lines. \n");
		}
	}

	//Convert to a single mesh

	printf("Extracting Mesh...\n");
	g_cpuPoly.extractSingleMeshObject();
	return true;
}


//Clean up
void Close() {
	SAFE_DELETE(g_lpGPUPoly);

	g_cpuPoly.cleanup();
	SAFE_DELETE(g_taskSchedular);
	//Delete Texture
	if(glIsTexture(g_textureId))
		glDeleteTextures(1, &g_textureId);
}


void ProduceUsageChartAsTexture(const char* chrDBPath,
							    const PolyMPUs& polyMPUs,
							    MPUSTATS* lpMPUStats,
							    tick_count& t0, tick_count& t1,
							    int xpID, int ctThreads,
							    float offsetPercentFromEnd)
{
	//FrameBuffer and Render Buffer Object Ids
	GLuint fboId;
	GLuint rboId;
	Clamp<float>(offsetPercentFromEnd, 0.0f, 1.0f);

	//Generate FrameBuffer
	glGenFramebuffers(1, &fboId);
	glBindFramebuffer(GL_FRAMEBUFFER, fboId);

	//Generate FrameBufferMPUSTATS
	glGenRenderbuffers(1, &rboId);
	glBindRenderbuffer(GL_FRAMEBUFFER, rboId);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, WINDOW_SIZE_WIDTH, WINDOW_SIZE_HEIGHT);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);

	// attach a texture to FBO color attachement point
    // create a texture object
    glGenTextures(1, &g_textureId);
    glBindTexture(GL_TEXTURE_2D, g_textureId);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, WINDOW_SIZE_WIDTH, WINDOW_SIZE_HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    //
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g_textureId, 0);

	// attach a renderbuffer to depth attachment point
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rboId);

//	if(!CheckFramebufferStatus())
	//	return;

	// adjust viewport and projection matrix to texture dimension
	glViewport(0, 0, WINDOW_SIZE_WIDTH, WINDOW_SIZE_HEIGHT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0f, WINDOW_SIZE_WIDTH, 0.0f, WINDOW_SIZE_HEIGHT, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);

	//Get All Info
	U32 statsThreadProcessed[MAX_THREADS_COUNT];
	U32 statsThreadCrossed[MAX_THREADS_COUNT];
	U32 statsThreadEmpty[MAX_THREADS_COUNT];
	double statsThreadLatestTime[MAX_THREADS_COUNT];
	double statsThreadCrossedTime[MAX_THREADS_COUNT];
	double statsThreadEmptyTime[MAX_THREADS_COUNT];
	memset(statsThreadProcessed, 0, sizeof(int) * MAX_THREADS_COUNT);
	memset(statsThreadCrossed, 0, sizeof(int) * MAX_THREADS_COUNT);
	memset(statsThreadEmpty, 0, sizeof(int) * MAX_THREADS_COUNT);

	memset(statsThreadLatestTime, 0, sizeof(double) * MAX_THREADS_COUNT);
	memset(statsThreadCrossedTime, 0, sizeof(double) * MAX_THREADS_COUNT);
	memset(statsThreadEmptyTime, 0, sizeof(double) * MAX_THREADS_COUNT);


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

	//Render with FBP
	// set the rendering destination to FBO
	glBindFramebuffer(GL_FRAMEBUFFER, fboId);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();
	glDisable(GL_LIGHTING);
		glPushAttrib(GL_ALL_ATTRIB_BITS);
			glColor3f(1.0f, 0.0f, 1.0f);
			glBegin(GL_QUADS);
				glVertex2f(0.0, 0.0f);
				glVertex2f(1.0, 0.0f);
				glVertex2f(1.0, 1.0f);
				glVertex2f(0.0, 1.0f);
			glEnd();

			double wTime = (t1 - t0).seconds() * offsetPercentFromEnd;
			double soff	 = (t1 - t0).seconds() * (1.0f - offsetPercentFromEnd);


			double wTimeUnit = WINDOW_SIZE_WIDTH / wTime;
			Clamp<double>(wTimeUnit, 0.0f, WINDOW_SIZE_WIDTH);

			double hTimeUnit = (WINDOW_SIZE_HEIGHT / (double)ctThreads);
			double hTimeStack = 0.75 * hTimeUnit;
			//double hGap = 0.25 * hTimeUnit;

			//1.Draw Boundary Lines in BLACK
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glColor3f(0.0f, 0.0f, 0.0f);
			glBegin(GL_LINES);
				glVertex2d(0.0, 0.0);
				glVertex2d(0.0, 1.0);

				glVertex2d(wTime * wTimeUnit, 0.0f);
				glVertex2d(wTime * wTimeUnit, 1.0f);
			glEnd();
			glPopAttrib();

			//2.Draw a quad per each MPU (Crossed Blue, Non-Crossed RED)
			U32 ctWorkUnits = polyMPUs.countWorkUnits();
			for(U32 i=0; i < ctWorkUnits; i++)
			{
				if(lpMPUStats[i].bIntersected)
					glColor3f(0.0f, 0.0f, 1.0f);
				else
					glColor3f(1.0f, 0.0f, 0.0f);

				double xStart = ((lpMPUStats[i].tickStart - t0).seconds() - soff) * wTimeUnit;
				double xEnd = ((lpMPUStats[i].tickEnd - t0).seconds() - soff) * wTimeUnit;
				double yLo = lpMPUStats[i].idxThread * hTimeUnit;
				double yHi = yLo + hTimeStack;

				glBegin(GL_QUADS);
				glVertex2d(xStart, yLo);
				glVertex2f(xEnd, yLo);
				glVertex2f(xEnd, yHi);
				glVertex2f(xStart, yHi);
				glEnd();
			}

			//3. Write All Text Strings
			double hTimeStackHalf = hTimeStack * 0.5;
			vec4f black = Color::black().toVec4f();
			void* font = GLUT_BITMAP_8_BY_13;
			char buffer[1024];

			int avgWork = 0;
			for(int i=0; i<ctThreads; i++)
				avgWork += statsThreadProcessed[i];
			avgWork /= ctThreads;


			double tt = (t1 - t0).seconds() * 1000.0;
			sprintf(buffer, "Total Processed %u, Crossed %d, Fastest %d, Slowest %d, PolyT %.2f ms", polyMPUs.countWorkUnits(), ctIntersected, idxFastestThread + 1, idxLatestThread + 1, tt);
			DrawString(buffer, 5, WINDOW_SIZE_HEIGHT - 20, black.ptr(), font);
			for(int i=0; i<ctThreads; i++)
			{
				float ratio = (statsThreadProcessed[i] != 0)? (float)(statsThreadCrossed[i] * 100.0f) / (float)statsThreadProcessed[i] : 0.0f;
				double dif = (etLatest - statsThreadLatestTime[i]) * 1000.0;
				statsThreadEmpty[i] = statsThreadProcessed[i] - statsThreadCrossed[i];
				sprintf(buffer, "Thread %d:[T %u, X %u, NX %u %.2f%], [X %.2f, NX %.2f ms], TTF %.2f ms]",
						i+1, statsThreadProcessed[i], statsThreadCrossed[i], statsThreadEmpty[i], ratio,
						statsThreadCrossedTime[i] * 1000.0, statsThreadEmptyTime[i] * 1000.0, dif);
				DrawString(buffer, 5,  hTimeStackHalf + i * hTimeUnit, &black.x, font);

				if(g_appSettings.bDrawUtilizationCellProcessInfo)
				{
					//Show Average Cell Processing Info
					sprintf(buffer, "Avg MPU Time: X %.2f, NX %.2f ms, ImbalanceFactor MPU %.2f",
							(float)(statsThreadCrossedTime[i] * 1000.0) /(float)statsThreadCrossed[i],
							(float)(statsThreadEmptyTime[i] * 1000.0) / (float)statsThreadEmpty[i],
							(float)(statsThreadProcessed[i]) / (float)avgWork);
					DrawString(buffer, 5,  hTimeStackHalf - 14 + i * hTimeUnit, &black.x, font);
				}
			}

		glPopAttrib();
	glEnable(GL_LIGHTING);
	glPopMatrix();

	// back to normal window-system-provided framebuffer
	glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0); // unbind

	// trigger mipmaps generation explicitly
	// NOTE: If GL_GENERATE_MIPMAP is set to GL_TRUE, then glCopyTexSubImage2D()
	// triggers mipmap generation automatically. However, the texture attached
	// onto a FBO should generate mipmaps manually via glGenerateMipmapEXT().
	glBindTexture(GL_TEXTURE_2D, g_textureId);
	glGenerateMipmap(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);

	glDeleteFramebuffersEXT(1, &fboId);
	glDeleteRenderbuffersEXT(1, &rboId);

	//Insert Log Record
	sqlite_InsertUtilRecord(chrDBPath,
							xpID, ctThreads,
							statsThreadCrossed, statsThreadEmpty,
							statsThreadCrossedTime, statsThreadEmptyTime, statsThreadLatestTime,
							idxFastestThread, idxLatestThread);
	//Save FrameBuffer Texture to an image format

}

