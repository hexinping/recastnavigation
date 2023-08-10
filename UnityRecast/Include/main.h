
#pragma once


#define DLLEXPORT_API extern "C" __declspec(dllexport)

//加载导航文件
DLLEXPORT_API bool LoadNavMesh(const char* navmesh_path);

//射线检测碰撞点
DLLEXPORT_API bool Raycast(float* m_spos, float* m_epos, float* m_straightPath);

//寻路
DLLEXPORT_API bool FindPath(char* navmesh_path);
