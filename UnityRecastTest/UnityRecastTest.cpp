﻿// UnityRecastTest.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <main.h>




int main()
{
    std::cout << "Hello World!\n";

	bool isLoad = LoadNavMesh("../RecastDemo/Bin/solo_navmesh.bin");
	if (!isLoad)
	{
		printf("文件加载失败=============");
	}

	float start[3];
	float end[3];

	/*start[0] = 51.938f;
	start[1] = 0.0f;
	start[2] = 4.212f;

	end[0] = 50.910f;
	end[1] = 0.0f;
	end[2] = 15.596f;

	float hitpos[3];
	bool isHit = Raycast(start, end, hitpos);
	if (isHit)
	{
		printf("碰撞点 %.3f  %.3f  %.3f=============", hitpos[0], hitpos[1], hitpos[2]);
	}*/

	/*start[0] = 53.556f;
	start[1] = 0.0f;
	start[2] = 3.589f;

	end[0] = 49.368f;
	end[1] = 0.0f;
	end[2] = -21.540f;*/

	start[0] = 100.992f;
	start[1] = 0.0f;
	start[2] = 17.095f;

	end[0] = 63.017f;
	end[1] = 0.0f;
	end[2] = 17.095f;

	float pathPoint[2048 * 3];
	int pointCount;

	//3个数值算一个点,如果到终点之间有阻碍最后的点会靠近阻碍点
	bool isFind = FindPath(start, end, pathPoint, &pointCount);
	printf("路点个数：%d \n", pointCount);
	if (isFind && pointCount > 0)
	{
		for (int i = 0; i < pointCount; i++)
		{
			printf("路点 %.3f  %.3f  %.3f=============\n", pathPoint[3*i], pathPoint[3 * i+1],pathPoint[3 * i+2]);
		}
	}


	return 0;
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
