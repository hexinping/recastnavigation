#include "main.h"

#include <vector>
#include <string>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>
#include <DetourCommon.h>

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

static dtNavMesh* m_navMesh;
static dtNavMeshQuery* m_navQuery;
static dtQueryFilter m_filter;  //过滤器

static bool is_init_filter = false;
void InitFilter()
{
	if (is_init_filter)
		return;
	is_init_filter = true;
	m_filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED);
	m_filter.setExcludeFlags(0);
}

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;
static const int MAX_POLYS = 256;

dtNavMesh* LoadFile(const char* path)
{
	InitFilter();

	FILE* fp = fopen(path, "rb");
	if (!fp) return 0;

	// Read header.
	NavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (readLen != 1)
	{
		fclose(fp);
		return 0;
	}
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		return 0;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		return 0;
	}

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		fclose(fp);
		return 0;
	}
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return 0;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (readLen != 1)
		{
			fclose(fp);
			return 0;
		}

		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		readLen = fread(data, tileHeader.dataSize, 1, fp);
		if (readLen != 1)
		{
			dtFree(data);
			fclose(fp);
			return 0;
		}

		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

	fclose(fp);

	return mesh;
}

//加载导航文件
bool LoadNavMesh(const char* navmesh_path)
{
	dtFreeNavMesh(m_navMesh);
	m_navMesh = LoadFile(navmesh_path);
	if (!m_navMesh)
		return false;
	if (m_navQuery == NULL)
	{
		m_navQuery = new dtNavMeshQuery();
	}
	dtStatus ret = m_navQuery->init(m_navMesh, 2048);
	return ret == DT_SUCCESS;
}


//射线检测碰撞点
bool Raycast(float* m_spos, float* m_epos, float* m_hitPos)
{
	bool m_hitResult = false;

	float m_polyPickExt[3];
	m_polyPickExt[0] = 2;
	m_polyPickExt[1] = 4;
	m_polyPickExt[2] = 2;

	dtPolyRef m_startRef;
	dtPolyRef m_endRef;

	if (m_navQuery == NULL)
	{
		m_navQuery = new dtNavMeshQuery();
	}

	m_navQuery->findNearestPoly(m_spos, m_polyPickExt, &m_filter, &m_startRef, 0);
	m_navQuery->findNearestPoly(m_epos, m_polyPickExt, &m_filter, &m_endRef, 0);
	int m_nstraightPath = 0;
	if (m_startRef)
	{
		float t = 0;
		int m_npolys = 0;
		m_nstraightPath = 2;

		//m_straightPath[0] = m_spos[0];
		//m_straightPath[1] = m_spos[1];
		//m_straightPath[2] = m_spos[2];

		float m_hitNormal[3];
		//float m_hitPos[3];

		dtPolyRef m_polys[MAX_POLYS];
		m_navQuery->raycast(m_startRef, m_spos, m_epos, &m_filter, &t, m_hitNormal, m_polys, &m_npolys, MAX_POLYS);
		if (t > 1)
		{
			// No hit
			dtVcopy(m_hitPos, m_epos);
			m_hitResult = false;
		}
		else
		{
			// Hit
			dtVlerp(m_hitPos, m_spos, m_epos, t);
			m_hitResult = true;
		}
		//// Adjust height.
		if (m_npolys > 0)
		{
			float h = 0;
			m_navQuery->getPolyHeight(m_polys[m_npolys - 1], m_hitPos, &h);
			m_hitPos[1] = h;
		}
		//dtVcopy(m_straightPath, m_hitPos);
	}
	return m_hitResult;
}

//寻路
bool FindPath(char* navmesh_path)
{
	return true;
}

extern "C" {

	//自动build和Save 生成导航数据
	__declspec(dllexport) bool BuildAndSave(char* obj_path, char* navmesh_path, StructParam_Unity* params) {

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




