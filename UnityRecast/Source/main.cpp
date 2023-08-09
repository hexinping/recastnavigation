
#include <vector>
#include <string>

#include "imgui.h"
#include "imguiRenderGL.h"

#include "Recast.h"
#include "RecastDebugDraw.h"
#include "InputGeom.h"
#include "TestCase.h"
#include "Filelist.h"
#include "Sample_SoloMesh.h"
#include "Sample_TileMesh.h"
#include "Sample_TempObstacles.h"
#include "Sample_Debug.h"

#ifdef WIN32
#	define snprintf _snprintf
#	define putenv _putenv
#endif

using std::string;
using std::vector;

struct SampleItem
{
	Sample* (*create)();
	const string name;
};
Sample* createSolo() { return new Sample_SoloMesh(); }
Sample* createTile() { return new Sample_TileMesh(); }
Sample* createTempObstacle() { return new Sample_TempObstacles(); }
Sample* createDebug() { return new Sample_Debug(); }
static SampleItem g_samples[] =
{
	{ createSolo, "Solo Mesh" },
	{ createTile, "Tile Mesh" },
	{ createTempObstacle, "Temp Obstacles" },
};
static const int g_nsamples = sizeof(g_samples) / sizeof(SampleItem);


extern "C" {

	//自动build和Save 生成导航数据
	_declspec(dllexport) bool BuildAndSave(char* obj_path, char* navmesh_path, StructParam_Unity* params) {

		BuildContext* ctx = new BuildContext;
		Sample* sample = g_samples[0].create();
		sample->setContext(ctx);
		//1 load model
		InputGeom* geom = new InputGeom;
		geom->load(ctx, obj_path);
		sample->handleMeshChanged(geom);

		//2 set params
		sample->setParamFromUnity(params);
		
		//3 build
		sample->handleBuild();

		//4 save to navmesh
		sample->handleSave(navmesh_path);

		delete sample;
		delete geom;
		delete ctx;
		return true;

	}


}