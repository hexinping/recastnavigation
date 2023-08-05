// Add by yaukey(yaukeywang@gmail.com) at 2019-02-19.
// Declaration of all NavMesh things for unity.

#ifndef NAVMESH_EXPORTER_UNITY_H
#define NAVMESH_EXPORTER_UNITY_H

#include "Recast.h"
#include "DetourNavMesh.h"

#if defined(_MSC_VER) && defined(NAVMESH_EXPORT_API)
#define EXPORT_API __declspec(dllexport)
#else
#define EXPORT_API
#endif

// NavMesh filter flags.
enum UnityNavMeshPolyFlags
{
    UNM_POLYFLAGS_WALK = 0x01,      // Ability to walk (ground, grass, road)
    UNM_POLYFLAGS_SWIM = 0x02,      // Ability to swim (water).
    UNM_POLYFLAGS_DOOR = 0x04,      // Ability to move through doors.
    UNM_POLYFLAGS_JUMP = 0x08,      // Ability to jump.
    UNM_POLYFLAGS_DISABLED = 0x10,  // Disabled polygon
    UNM_POLYFLAGS_ALL = 0xffff      // All abilities.
};

//
// Save NavMesh to file, From recast demo.
//

// NavMesh mesh file format header.
static const int UNITY_NAVMESH_SET_MAGIC = ('M' << 24) | ('S' << 16) | ('E' << 8) | 'T';

// NavMesh mesh version.
static const int UNITY_NAVMESH_SET_VERSION = 1;

// Unity NavMesh set header info.
struct UnityNavMeshSetHeader
{
    // The file format header.
    int magic;

    // Data version.
    int version;

    // The min point of aabb bound.
    float boundBoxMin[3];

    // The max point of aabb bound.
    float boundBoxMax[3];

    // Number of tiles.
    int numTiles;

    // NavMesh create parameters.
    dtNavMeshParams params;

    // Default constructor.
    UnityNavMeshSetHeader() : magic(0), version(0), numTiles(0) {}
};

// Unity NavMesh tile header.
struct UnityNavMeshTileHeader
{
    // Tile reference.
    dtTileRef tileRef;

    // Tile data size.
    int dataSize;

    // Default constructor.
    UnityNavMeshTileHeader() : tileRef(0), dataSize(0) {}
};

// The core part of building NavMesh from exported unity NavMesh.
dtNavMesh* rcBuildNavMesh_Unity(
    rcContext* ctx, 
    const float* vertices, 
    int vertexCount, 
    const int* triangles, 
    int triangleCount, 
    unsigned short regionId, 
    const int* areas, 
    int maxVertexPerPolygon, 
    float cellSize, 
    float cellHeight, 
	float walkableHeight,
	float walkableRadius,
	float walkableClimb
);

extern "C"
{
    // The api used for unity to export NavMesh.
    EXPORT_API void NavMeshExporter_Unity(
        const float* vertices,
        int vertexCount,
        const int* trianglesIndices,
        int triangleIndexCount,
        unsigned short regionId,
        const int* area,
        float* boundMin,
        float* boundMax,
        float cellSize,
        float cellHeight,
        float walkableHeight,
        float walkableRadius,
        float walkableClimb,
        const char* exportPath
    );

	// The api used for importing NavMesh exported by NavMeshExporter_Unity from unity.
	EXPORT_API dtNavMesh* NavMeshImporter_Unity(const char* importPath, float* boundBoxMin, float* boundBoxMax);
}

#endif // NAVMESH_EXPORTER_UNITY_H
