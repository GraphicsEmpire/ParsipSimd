/*
 * SceneGraph.h
 *
 *  Created on: Nov 4, 2012
 *      Author: pourya
 */

#ifndef SCENEGRAPH_H_
#define SCENEGRAPH_H_

#include "tbb/tick_count.h"
#include "loki/Singleton.h"
#include "loki/Functor.h"
#include "loki/SmartPtr.h"
#include "base/CopyStack.h"
#include "base/AssetManager.h"
#include "base/FastAccessToNamedResource.h"
#include "base/MovingAverage.h"

#include "ShaderManager.h"
#include "TexManager.h"

#include "ArcBallCamera.h"
#include "SGNode.h"
#include "SGHeaders.h"

#ifdef PS_USE_BULLET
	#include "graphics/SGBulletRigidMesh.h"
	#include "graphics/SGBulletRigidDynamics.h"
	#include "graphics/SGBulletSoftRigidDynamics.h"
#endif


using namespace Loki;
using namespace std;
using namespace PS;
using namespace PS::GL;
using namespace PS::MATH;

#define FRAME_TIME_BUFFER_SIZE 32


namespace PS {
namespace SG {

struct LightSource {
	vec4f pos;
	vec4f color;
};

/*!
 * Scene Graph
 */
class SceneGraph {
public:
	SceneGraph();
	virtual ~SceneGraph();

    //Draws the entire scenegraph for traversing through the list of
    //nodes and calling individual draw methods
    void draw();
    void drawBBoxes();
    void timestep();

    //Nodes
    U32 add(SGNode* aNode);


    U32 addSceneBox(const AABB& box);
    U32 addFloor(int rows, int cols, float step = 1.0f);
    bool remove(U32 index);
    bool remove(const string& name);
    bool remove(const SGNode* pnode);

    U32 count() const {return (U32)m_vSceneNodes.size();}
    SGNode* get(U32 index) const {return m_vSceneNodes[index];}
    SGNode* get(const char* name) const;
    SGNode* last() const;



    //Matrix Stacks
	CopyStack<mat44f>& stkProjection() {return m_stkProjection;}
	CopyStack<mat44f>& stkModelView() {return m_stkModelView;}
	mat44f modelviewprojection() const;

    //Camera
    ArcBallCamera& camera() { return m_camera;}
    
    //Mouse calls
    void mousePress(int button, int state, int x, int y);
    void mouseWheel(int button, int dir, int x, int y);
    void mouseMove(int x, int y);
    
    //Coordinate Conversion
    bool screenToWorld(const vec3f& s, vec3f& w);
    Ray  screenToWorldRay(int x, int y);
    vec4i viewport() const;

    //Headers
    SGHeaders* headers() const {return m_headers;}
    void updateCameraHeader();

    //Modifier
    int getModifier() const {return m_keyModifier;}
    void setModifier(int mod) {m_keyModifier = mod;}

    void update();
    static AnsiStr gpuInfo();

    //print structure
    void print(const char* switches = "-a") const;

    //Save and Load view settings
	bool readConfig(const AnsiStr& strFP = "scene.ini");
	bool writeConfig(const AnsiStr& strFP = "scene.ini");

	//bullet
#ifdef PS_USE_BULLET
    U32 addRigidBody(SGBulletRigidMesh* aRigidBody);
    U32 addSoftBody(SGBulletSoftMesh* aSoftBody);
    SGBulletSoftRigidDynamics* world() { return m_lpWorld;}
#endif


    //Timing and Profiling services

    //Object Selection

    //Surfaces

    
protected:
    void cleanup();

private:
#ifdef PS_USE_BULLET
    SGBulletSoftRigidDynamics* m_lpWorld;
#endif

    int m_keyModifier;
	CopyStack<mat44f> m_stkProjection;
	CopyStack<mat44f> m_stkModelView;
	std::vector<SGNode*> m_vSceneNodes;


    ArcBallCamera m_camera;
    SGHeaders* m_headers;
    int m_idCamHeader;
    int m_idGPUHeader;
    int m_idAnimationHeader;

    //Stats
    MovingAvg<double, FRAME_TIME_BUFFER_SIZE> m_avgFrameTime;
    tbb::tick_count m_tick;
    double m_fps;
    U64 m_ctFrames;
    U64 m_ctSampledFrames;


};

//Singleton Access to scene graph
typedef SingletonHolder<SceneGraph, CreateUsingNew, PhoenixSingleton> TheSceneGraph;

}
}

#endif /* SCENEGRAPH_H_ */
