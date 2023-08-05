// Add by yaukey(yaukeywang@gmail.com) at 2019-02-19.
// Implementation of all recast mesh things for unity.

#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"
#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "NavMeshExporter_Unity.h"

#include <map>
#include <vector>
#include <unordered_map>

// Use the implementation in RecastMesh.cpp.
inline int prev_vertex(int i, int n) { return i - 1 >= 0 ? i - 1 : n - 1; }
inline int next_vertex(int i, int n) { return i + 1 < n ? i + 1 : 0; }

// Copy form rcEdge from RecastMesh.cpp, internal use only.
struct rcEdge_Unity
{
    unsigned short vert[2];
    unsigned short polyEdge[2];
    unsigned short poly[2];
};

// Copy form buildMeshAdjacency from RecastMesh.cpp, internal use only.
static bool buildMeshAdjacency_Unity(unsigned short* polys, const int npolys, const int nverts, const int vertsPerPoly)
{
    // Based on code by Eric Lengyel from:
    // http://www.terathon.com/code/edges.php

    int maxEdgeCount = npolys * vertsPerPoly;
    unsigned short* firstEdge = (unsigned short*)rcAlloc(sizeof(unsigned short)*(nverts + maxEdgeCount), RC_ALLOC_TEMP);
    if (!firstEdge)
        return false;
    unsigned short* nextEdge = firstEdge + nverts;
    int edgeCount = 0;

    rcEdge_Unity* edges = (rcEdge_Unity*)rcAlloc(sizeof(rcEdge_Unity)*maxEdgeCount, RC_ALLOC_TEMP);
    if (!edges)
    {
        rcFree(firstEdge);
        return false;
    }

    for (int i = 0; i < nverts; i++)
        firstEdge[i] = RC_MESH_NULL_IDX;

    for (int i = 0; i < npolys; ++i)
    {
        unsigned short* t = &polys[i*vertsPerPoly * 2];
        for (int j = 0; j < vertsPerPoly; ++j)
        {
            if (t[j] == RC_MESH_NULL_IDX) break;
            unsigned short v0 = t[j];
            unsigned short v1 = (j + 1 >= vertsPerPoly || t[j + 1] == RC_MESH_NULL_IDX) ? t[0] : t[j + 1];
            if (v0 < v1)
            {
                rcEdge_Unity& edge = edges[edgeCount];
                edge.vert[0] = v0;
                edge.vert[1] = v1;
                edge.poly[0] = (unsigned short)i;
                edge.polyEdge[0] = (unsigned short)j;
                edge.poly[1] = (unsigned short)i;
                edge.polyEdge[1] = 0;
                // Insert edge
                nextEdge[edgeCount] = firstEdge[v0];
                firstEdge[v0] = (unsigned short)edgeCount;
                edgeCount++;
            }
        }
    }

    for (int i = 0; i < npolys; ++i)
    {
        unsigned short* t = &polys[i*vertsPerPoly * 2];
        for (int j = 0; j < vertsPerPoly; ++j)
        {
            if (t[j] == RC_MESH_NULL_IDX) break;
            unsigned short v0 = t[j];
            unsigned short v1 = (j + 1 >= vertsPerPoly || t[j + 1] == RC_MESH_NULL_IDX) ? t[0] : t[j + 1];
            if (v0 > v1)
            {
                for (unsigned short e = firstEdge[v1]; e != RC_MESH_NULL_IDX; e = nextEdge[e])
                {
                    rcEdge_Unity& edge = edges[e];
                    if (edge.vert[1] == v0 && edge.poly[0] == edge.poly[1])
                    {
                        edge.poly[1] = (unsigned short)i;
                        edge.polyEdge[1] = (unsigned short)j;
                        break;
                    }
                }
            }
        }
    }

    // Store adjacency
    for (int i = 0; i < edgeCount; ++i)
    {
        const rcEdge_Unity& e = edges[i];
        if (e.poly[0] != e.poly[1])
        {
            unsigned short* p0 = &polys[e.poly[0] * vertsPerPoly * 2];
            unsigned short* p1 = &polys[e.poly[1] * vertsPerPoly * 2];
            p0[vertsPerPoly + e.polyEdge[0]] = e.poly[1];
            p1[vertsPerPoly + e.polyEdge[1]] = e.poly[0];
        }
    }

    rcFree(firstEdge);
    rcFree(edges);

    return true;
}

// edge index - (1st tri of the shared edge / 2nd tri of the shared edge).
// Define poly mesh edge shared info.
// key - edge hash, value - the two triangles that share this edge, if only one triangle then, the other value will be -1.
typedef std::map<int, std::pair<int, int> > PolyMeshEdgeSharedMap;

// The vertex position in vector space.
struct VertexPosition
{
    float position[3];

    VertexPosition()
    {
        position[0] = 0.0f;
        position[1] = 0.0f;
        position[2] = 0.0f;
    }

	VertexPosition(float x, float y, float z)
	{
		position[0] = x;
		position[1] = y;
		position[2] = z;
	}
};

// Hash function for vertex position map.
struct VertexPositionHashFunc
{
	size_t operator()(const VertexPosition& o) const
	{
        std::hash<float> hasher = std::hash<float>();
		return hasher(o.position[0]) + hasher(o.position[1]) + hasher(o.position[2]);
	}
};

// Compare function for vertex position map.
struct VertexPositionCompareFunc
{
	bool operator()(const VertexPosition& l, const VertexPosition& r) const
	{
		return (rcAbs(l.position[0] - r.position[0]) < 0.000001f) && (rcAbs(l.position[1] - r.position[1]) < 0.000001f) && (rcAbs(l.position[2] - r.position[2]) < 0.000001f);
	}
};

// Vertex position and index map.
// key - vertex in vector space, value - the vertex index in merged vertices buffer.
typedef std::unordered_map<VertexPosition, int, VertexPositionHashFunc, VertexPositionCompareFunc> VertexIndexMap;

// Build a poly mesh from unity nav mesh.
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
)
{
    rcAssert(ctx);
    rcAssert(vertices);
    rcAssert(triangles);
    rcAssert(areas);

    // Too many vertex.
    if (vertexCount >= 0xfffe)
    {
        ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: Too many vertices %d.", vertexCount);
		return NULL;
    }

    ctx->startTimer(RC_TIMER_TOTAL);

    // Calculate the AABB bound.
    float boundMin[3] = { 0.0f, 0.0f, 0.0f };
    float boundMax[3] = { 0.0f, 0.0f, 0.0f };

    rcCalcBounds(vertices, vertexCount, boundMin, boundMax);

    // Alloc temporary memory to store the merged vertex and index data.
	rcScopedDelete<float> tempRawVertices((float*)rcAlloc(sizeof(float) * vertexCount * 3, RC_ALLOC_TEMP));
	if (NULL == tempRawVertices)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: Out of memory 'tempRawVertices' (%d).", vertexCount * 3);
		return NULL;
	}

    rcScopedDelete<unsigned short> tempVoxelVertices((unsigned short*)rcAlloc(sizeof(unsigned short) * vertexCount * 3, RC_ALLOC_TEMP));
    if (NULL == tempVoxelVertices)
    {
        ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: Out of memory 'tempVoxelVertices' (%d).", vertexCount * 3);
		return NULL;
    }

    rcScopedDelete<unsigned short> mergeVertexMap((unsigned short*)rcAlloc(sizeof(unsigned short) * vertexCount, RC_ALLOC_TEMP));
    if (NULL == mergeVertexMap)
    {
        ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: Out of memory 'mergeVertexMap' (%d).", vertexCount);
		return NULL;
    }

    rcScopedDelete<unsigned short> tempPolys((unsigned short*)rcAlloc(sizeof(unsigned short) * triangleCount * maxVertexPerPolygon, RC_ALLOC_TEMP));
    if (NULL == tempPolys)
    {
        ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: Out of memory 'tempPolys' (%d).", triangleCount * maxVertexPerPolygon);
		return NULL;
    }

    rcScopedDelete<unsigned char> tempArea((unsigned char*)rcAlloc(sizeof(unsigned char) * triangleCount, RC_ALLOC_TEMP));
    if (NULL == tempArea)
    {
        ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: Out of memory 'tempArea' (%d).", triangleCount);
		return NULL;
    }

    // Group separated triangles by shared edge into a same poly.
	// Count how many polygons in total.
	int polyCount = 0;
    {
        // Build shared edge info.
        PolyMeshEdgeSharedMap sharedEdgeMap;
        for (int i = 0; i < triangleCount; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                int va = triangles[i * 3 + j];
                int vb = triangles[i * 3 + next_vertex(j, 3)];
                if (va > vb)
                {
                    rcSwap(va, vb);
                }

                // Check validation.
                rcAssert(va < 0x10000 && vb < 0x10000);

                // Check edge map, bind two vertex(an edge) to be the edge index.
                int edge = (va << 16) | vb;
                PolyMeshEdgeSharedMap::iterator ifind = sharedEdgeMap.find(edge);
                if (ifind != sharedEdgeMap.end())
                {
                    // Find the shared edge with other triangle, tag its triangle index.
                    ifind->second.second = i;
                }
                else
                {
                    // Just tag this triangle index.
                    sharedEdgeMap[edge] = std::make_pair(i, -1);
                }
            }
        }

        // Merge triangle by same shared edge.
        rcScopedDelete<bool> triangleUsedMask((bool*)rcAlloc(sizeof(bool) * triangleCount, RC_ALLOC_TEMP));
        if (NULL == triangleUsedMask)
        {
            ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: Out of memory 'pTriangleMark' (%d).", triangleCount);
			return NULL;
        }

        memset(triangleUsedMask, 0, sizeof(bool) * triangleCount);

        // Get every polygon wrapped by separated vertex in the same position.
		// vertex index / edge checked.
        std::vector<std::pair<int, bool> > polyVertexCollector;
        for (int triIdx = 0; triIdx < triangleCount; triIdx++)
        {
            // Already checked this triangle.
            if (triangleUsedMask[triIdx])
            {
                continue;
            }

            // Tag this triangle has checked.
            triangleUsedMask[triIdx] = true;

            // Clear the temp collected poly and begin to find new poly.
            polyVertexCollector.clear();
            for (int vertInTri = 0; vertInTri < 3; vertInTri++)
            {
                // Fill current triangle to be the basic poly.
                polyVertexCollector.push_back(std::make_pair(triangles[triIdx * 3 + vertInTri], true));
            }

            // Find triangle by shared edge.
            while (true)
            {
                // Assume we have no new edge.
                bool findNewEdge = false;

                // Check every edge of current poly if it has any shared-edge triangle to merge.
                for (int vertIdxInPoly = 0; vertIdxInPoly < (int)polyVertexCollector.size(); vertIdxInPoly++)
                {
                    // We have already checked this vertex.
                    if (!polyVertexCollector[vertIdxInPoly].second)
                    {
                        continue;
                    }

                    // Tag this vertex we have checked.
                    polyVertexCollector[vertIdxInPoly].second = false;

                    // Check edge formed by this and next vertex.
                    int va = polyVertexCollector[vertIdxInPoly].first;
                    int vb = polyVertexCollector[next_vertex(vertIdxInPoly, (int)polyVertexCollector.size())].first;
                    if (va > vb)
                    {
                        rcSwap(va, vb);
                    }

                    // Check validation.
                    rcAssert(va < 0x10000 && vb < 0x10000);

                    // From edge and get the shared edge info.
                    int edge = (va << 16) | vb;
                    PolyMeshEdgeSharedMap::mapped_type& sharedEdgeInfo = sharedEdgeMap[edge];
                    if (sharedEdgeInfo.second < 0)
                    {
                        // No triangle share this edge.
                        continue;
                    }

                    // Check the shared-edge triangle if it has not done.
                    int tri = triangleUsedMask[sharedEdgeInfo.first] ? sharedEdgeInfo.second : sharedEdgeInfo.first;
                    if (triangleUsedMask[tri])
                    {
                        // All of them has already been checked.
                        continue;
                    }

                    // Tag has checked this triangle, and means a new edge has come into this poly.
                    triangleUsedMask[tri] = true;
                    findNewEdge = true;
                    for (int vertInNewTri = 0; vertInNewTri < 3; vertInNewTri++)
                    {
                        int nv = triangles[tri * 3 + vertInNewTri];
                        if ((nv == va) || (nv == vb))
                        {
                            continue;
                        }

                        // Get previous and next of current vertex vertIdxInPoly.
                        int vertexPrev = prev_vertex(vertIdxInPoly, (int)polyVertexCollector.size());
                        int vertexNext = next_vertex(vertIdxInPoly, (int)polyVertexCollector.size());
                        int vertexNNext = next_vertex(vertexNext, (int)polyVertexCollector.size());

                        // Check each condition.
                        if (polyVertexCollector[vertexPrev].first == nv)
                        {
                            // This means current polygon is concave, not normal.
                            // merge edge.
                            polyVertexCollector.erase(polyVertexCollector.begin() + vertIdxInPoly);
                            polyVertexCollector[vertexPrev].second = true;
                            vertIdxInPoly--;
                        }
                        else if (polyVertexCollector[vertexNNext].first == nv)
                        {
                            // This means current polygon is concave, not normal.
                            // Merge edge.
                            polyVertexCollector.erase(polyVertexCollector.begin() + vertexNext);
                            polyVertexCollector[vertexNNext].second = true;
                            vertIdxInPoly--;
                        }
                        else
                        {
                            // This means normal condition.
                            // We insert a new edge.
                            polyVertexCollector.insert(polyVertexCollector.begin() + vertIdxInPoly + 1, std::make_pair(nv, true));
                            polyVertexCollector[vertIdxInPoly].second = true;
                            vertIdxInPoly++;
                        }

                        break;
                    }
                }

                // Break if no new edge found.
                if (!findNewEdge)
                {
                    break;
                }
            }

            // Assign poly data.
            unsigned short* curPoly = tempPolys + polyCount * maxVertexPerPolygon;
            for (int i = 0; i < (int)polyVertexCollector.size(); i++)
            {
                curPoly[i] = (unsigned short)polyVertexCollector[i].first;
            }

            for (int j = (int)polyVertexCollector.size(); j < maxVertexPerPolygon; j++)
            {
                curPoly[j] = RC_MESH_NULL_IDX;
            }

            tempArea[polyCount] = (unsigned char)(areas[triIdx]);
            polyCount++;
        }
    }

    // Start to merge vertex.
	int mergedVertexCount = 0;
	{
		VertexIndexMap vertexIndexMap;
		for (int i = 0; i < vertexCount; i++)
		{
			const float* baseVertex = vertices + i * 3;

			// Fill the vertex position.
			VertexPosition vertInVoxel(baseVertex[0], baseVertex[1], baseVertex[2]);

			// Find and merge same vertex.
			VertexIndexMap::iterator vertIdxIter = vertexIndexMap.find(vertInVoxel);
			if (vertIdxIter == vertexIndexMap.end())
			{
				// Add normal vertex index.
				vertexIndexMap[vertInVoxel] = mergedVertexCount;

				// Copy raw position.
				memcpy(tempRawVertices + mergedVertexCount * 3, vertInVoxel.position, sizeof(vertInVoxel.position));

				// Copy voxel position.
				unsigned short vertexVoxel[3];
				vertexVoxel[0] = (unsigned short)((baseVertex[0] - boundMin[0]) / cellSize);
				vertexVoxel[1] = (unsigned short)((baseVertex[1] - boundMin[1]) / cellHeight);
				vertexVoxel[2] = (unsigned short)((baseVertex[2] - boundMin[2]) / cellSize);
				memcpy(tempVoxelVertices + mergedVertexCount * 3, vertexVoxel, sizeof(vertexVoxel));

				// Add vertex index.
				mergeVertexMap[i] = mergedVertexCount;
				mergedVertexCount++;
			}
			else
			{
				// Get index of the pre-saved same vertex.
				mergeVertexMap[i] = vertIdxIter->second;
			}
		}
    }

	// Alloc poly mesh.
	rcPolyMesh* polyMesh = rcAllocPolyMesh();
	if (NULL == polyMesh)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: Out of memory rcAllocPolyMesh.");
		return NULL;
	}

    // Fill bound box.
    rcVcopy(polyMesh->bmin, boundMin);
    rcVcopy(polyMesh->bmax, boundMax);

	// Fill some other parameters.
    polyMesh->nverts = mergedVertexCount;
    polyMesh->npolys = polyCount;
    polyMesh->maxpolys = polyCount;
    polyMesh->nvp = maxVertexPerPolygon;
    polyMesh->cs = cellSize;
    polyMesh->ch = cellHeight;
    polyMesh->borderSize = 0;

    // Alloc poly mesh vertex, polygons, region, flag and area.
    polyMesh->verts = (unsigned short*)rcAlloc(sizeof(unsigned short) * mergedVertexCount * 3, RC_ALLOC_PERM);
    if (NULL == polyMesh->verts)
    {
        ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: Out of memory 'polyMesh->verts' (%d).", mergedVertexCount * 3);
		rcFreePolyMesh(polyMesh);
		return NULL;
    }

    polyMesh->polys = (unsigned short*)rcAlloc(sizeof(unsigned short) * polyCount * maxVertexPerPolygon * 2, RC_ALLOC_PERM);
    if (NULL == polyMesh->polys)
    {
        ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: Out of memory 'polyMesh->polys' (%d).", polyCount * maxVertexPerPolygon * 2);
		rcFreePolyMesh(polyMesh);
		return NULL;
    }

    polyMesh->regs = (unsigned short*)rcAlloc(sizeof(unsigned short) * polyCount, RC_ALLOC_PERM);
    if (NULL == polyMesh->regs)
    {
        ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: Out of memory 'polyMesh->regs' (%d).", polyCount);
		rcFreePolyMesh(polyMesh);
		return NULL;
    }

    polyMesh->flags = (unsigned short*)rcAlloc(sizeof(unsigned short) * polyCount, RC_ALLOC_PERM);
    if (NULL == polyMesh->flags)
    {
        ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: Out of memory 'polyMesh->flags' (%d).", polyCount);
		rcFreePolyMesh(polyMesh);
		return NULL;
    }

    polyMesh->areas = (unsigned char*)rcAlloc(sizeof(unsigned char) * polyCount, RC_ALLOC_PERM);
    if (NULL == polyMesh->areas)
    {
        ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: Out of memory 'polyMesh->areas' (%d).", polyCount);
		rcFreePolyMesh(polyMesh);
        return NULL;
    }

    // assign values.
    memcpy(polyMesh->verts, tempVoxelVertices, sizeof(unsigned short) * mergedVertexCount * 3);
    memset(polyMesh->flags, 0, sizeof(unsigned short) * polyCount);
    memset(polyMesh->polys, 0xff, sizeof(unsigned short) * polyCount * maxVertexPerPolygon * 2);
    memcpy(polyMesh->areas, tempArea, sizeof(unsigned char) * polyCount);

	// Copy all poly vertex into poly mesh struct.
    for (int i = 0; i < polyCount; i++)
    {
        unsigned short* srcPoly = tempPolys + i * maxVertexPerPolygon;
        unsigned short* dstPoly = polyMesh->polys + i * maxVertexPerPolygon * 2;
        for (int j = 0; j < maxVertexPerPolygon; j++)
        {
            // Get vertex read vertex index.
			// We update vertex index to merged index here.
            dstPoly[j] = (srcPoly[j] != RC_MESH_NULL_IDX) ? mergeVertexMap[srcPoly[j]] : RC_MESH_NULL_IDX;
        }

        polyMesh->regs[i] = regionId;
    }

    // Build adjacency info.
    buildMeshAdjacency_Unity(polyMesh->polys, polyMesh->npolys, polyMesh->nverts, maxVertexPerPolygon);

    // Check vertex count.
    if (polyMesh->nverts > 0xffff)
    {
        ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: The resulting mesh has too many vertices %d (max %d). Data can be corrupted.", polyMesh->nverts, 0xffff);
		rcFreePolyMesh(polyMesh);
		return NULL;
    }

    // Check polygon count.
    if (polyMesh->npolys > 0xffff)
    {
        ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: The resulting mesh has too many polygons %d (max %d). Data can be corrupted.", polyMesh->npolys, 0xffff);
		rcFreePolyMesh(polyMesh);
		return NULL;
    }

	// Update poly mesh flags.
	for (int i = 0; i < polyMesh->npolys; i++)
	{
		polyMesh->flags[i] = UNM_POLYFLAGS_ALL ^ UNM_POLYFLAGS_DISABLED;
	}

	// Build poly mesh finished, and then use the information to build nav mesh.

	// Fill nav mesh create parameters.
	dtNavMeshCreateParams nmParams;
	memset(&nmParams, 0, sizeof(nmParams));

	// Fill base data.
	nmParams.verts = polyMesh->verts;
	nmParams.vertCount = polyMesh->nverts;
	nmParams.polys = polyMesh->polys;
	nmParams.polyAreas = polyMesh->areas;
	nmParams.polyFlags = polyMesh->flags;
	nmParams.polyCount = polyMesh->npolys;
	nmParams.nvp = polyMesh->nvp;

	// Fill agent data.
	nmParams.walkableHeight = walkableHeight;
	nmParams.walkableRadius = walkableRadius;
	nmParams.walkableClimb = walkableClimb;

	// Fill bound box.
	rcVcopy(nmParams.bmin, polyMesh->bmin);
	rcVcopy(nmParams.bmax, polyMesh->bmax);

	// Fill cell size.
	nmParams.cs = polyMesh->cs;
	nmParams.ch = polyMesh->ch;
	nmParams.buildBvTree = true;

	// Create nav mesh data.
	unsigned char* navMeshData = NULL;
	int navMeshDataSize = 0;
	if (!dtCreateNavMeshData(&nmParams, &navMeshData, &navMeshDataSize))
	{
		ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: Out of memory dtCreateNavMeshData.");
		rcFreePolyMesh(polyMesh);
		return NULL;
	}

	// Because transform vertex position from vector space into voxel space will cause data loss,
	// we may lost some vertex when transforming back in creating nav mesh data, so we copy vertex position
	// back form the raw vertex position.
	{
		// Calculate data size.
		unsigned char* nmd = navMeshData;
		const int headerSize = dtAlign4(sizeof(dtMeshHeader));
		const int vertsSize = dtAlign4(sizeof(float) * 3 * nmParams.vertCount);

		// Move the pointer to the position we want.
		dtGetThenAdvanceBufferPointer<dtMeshHeader>(nmd, headerSize);
		float* navVerts = dtGetThenAdvanceBufferPointer<float>(nmd, vertsSize);

		// We then copy the origin vertex position back here.
		for (int i = 0; i < nmParams.vertCount; ++i)
		{
			float* srcV = &tempRawVertices[i * 3];
			float* dstV = &navVerts[i * 3];
			dstV[0] = srcV[0];
			dstV[1] = srcV[1];
			dstV[2] = srcV[2];
		}
	}

	// Alloc nav mesh.
	dtNavMesh* navMesh = dtAllocNavMesh();
	if (NULL == navMesh)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildNavMesh_Unity: Out of memory dtAllocNavMesh.");
		rcFreePolyMesh(polyMesh);
		return NULL;
	}

	// Init nav mesh from nav mesh data.
	if (dtStatusFailed(navMesh->init(navMeshData, navMeshDataSize, DT_TILE_FREE_DATA)))
	{
		dtFreeNavMesh(navMesh);
		rcFreePolyMesh(polyMesh);
		return NULL;
	}

	ctx->stopTimer(RC_TIMER_TOTAL);
    return navMesh;
}

// Export nav mesh for unity.
void NavMeshExporter_Unity(
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
)
{
	// Build nav mesh.
    rcContext buildContext;
	dtNavMesh* navMesh = rcBuildNavMesh_Unity(
		&buildContext,
		vertices,
		vertexCount,
		trianglesIndices,
		triangleIndexCount / 3,
		regionId,
		area,
		DT_VERTS_PER_POLYGON, 
		cellSize,
		cellHeight,
		walkableHeight,
		walkableRadius,
		walkableClimb
	);
    if (NULL == navMesh)
    {
        return;
    }

    // Calculate the AABB bound.
    float boundMinPosition[3] = { 0.0f, 0.0f, 0.0f };
    float boundMaxPosition[3] = { 0.0f, 0.0f, 0.0f };
    rcCalcBounds(vertices, vertexCount, boundMinPosition, boundMaxPosition);

    // Save nav mesh to file.
    FILE *file = fopen(exportPath, "wb");
    if (file == NULL)
    {
        return;
    }

    // Store header.
    UnityNavMeshSetHeader fileHeader;
    fileHeader.magic = UNITY_NAVMESH_SET_MAGIC;
    fileHeader.version = UNITY_NAVMESH_SET_VERSION;

    // Store the aabb bound.
    dtVcopy(fileHeader.boundBoxMin, boundMinPosition);
    dtVcopy(fileHeader.boundBoxMax, boundMaxPosition);

    // Copy to the out result.
    dtVcopy(boundMin, boundMinPosition);
    dtVcopy(boundMax, boundMaxPosition);

    // Get tile count.
    const dtNavMesh *saveMesh = navMesh;
    fileHeader.numTiles = 0;
    for (int i = 0; i < saveMesh->getMaxTiles(); i++)
    {
        const dtMeshTile* meshTile = saveMesh->getTile(i);
        if ((NULL == meshTile) || (NULL == meshTile->header) || (meshTile->dataSize <= 0))
        {
            continue;
        }

        fileHeader.numTiles++;
    }

    // Store header params.
    memcpy(&fileHeader.params, saveMesh->getParams(), sizeof(dtNavMeshParams));
    fwrite(&fileHeader, sizeof(fileHeader), 1, file);

    // Store tiles.
    for (int i = 0; i < saveMesh->getMaxTiles(); i++)
    {
        const dtMeshTile* meshTile = saveMesh->getTile(i);
        if ((NULL == meshTile) || (NULL == meshTile->header) || (meshTile->dataSize <= 0))
        {
            continue;
        }

        // Store tile header.
        UnityNavMeshTileHeader tileHeader;
        tileHeader.tileRef = saveMesh->getTileRef(meshTile);
        tileHeader.dataSize = meshTile->dataSize;

        fwrite(&tileHeader, sizeof(tileHeader), 1, file);

        // Store tile data.
        fwrite(meshTile->data, meshTile->dataSize, 1, file);
    }

    fclose(file);
    file = NULL;

    // Free memory.
    dtFreeNavMesh(navMesh);
}

// Import nav mesh from unity.
dtNavMesh* NavMeshImporter_Unity(const char* importPath, float* boundBoxMin, float* boundBoxMax)
{
	if (NULL == importPath)
	{
		return NULL;
	}

	FILE* file = fopen(importPath, "rb");
	if (NULL == file)
	{
		return NULL;
	}

	// Read header.
	UnityNavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(UnityNavMeshSetHeader), 1, file);
	if (1 != readLen)
	{
		fclose(file);
		return NULL;
	}

	if (UNITY_NAVMESH_SET_MAGIC != header.magic)
	{
		fclose(file);
		return NULL;
	}

	if (UNITY_NAVMESH_SET_VERSION != header.version)
	{
		fclose(file);
		return NULL;
	}

	if (header.numTiles <= 0)
	{
		fclose(file);
		return NULL;
	}

	// Read bound box if you need.
	dtVcopy(boundBoxMin, header.boundBoxMin);
	dtVcopy(boundBoxMax, header.boundBoxMax);

	// Alloc nav mesh.
	dtNavMesh* navMesh = dtAllocNavMesh();
	if (NULL == navMesh)
	{
		fclose(file);
		return NULL;
	}

	// Init nav mesh from params.
	dtStatus status = navMesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(file);
		return NULL;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		UnityNavMeshTileHeader tileHeader;
		readLen = fread(&tileHeader, sizeof(tileHeader), 1, file);
		if (1 != readLen)
		{
			fclose(file);
			return NULL;
		}

		if ((0 == tileHeader.tileRef) || (tileHeader.dataSize <= 0))
		{
			break;
		}

		unsigned char* tileData = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (NULL == tileData)
		{
			break;
		}

		memset(tileData, 0, tileHeader.dataSize);
		readLen = fread(tileData, tileHeader.dataSize, 1, file);
		if (1 != readLen)
		{
			dtFree(tileData);
			fclose(file);
			return NULL;
		}

		navMesh->addTile(tileData, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

	fclose(file);
	return navMesh;
}
