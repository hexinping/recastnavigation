
--[==[

Unity的NavMesh数据都是基于三角面的，里面的点数据有位置相同的，需要自己合并


]==]


--[==[

工作流：要实现客户端和服务器寻路算法一致，需要保证寻路导航数据以及寻路算法一致

常用做法
第一种 自己写寻路算法
{
   1 客户端利用Unity的NavMesh的烘焙数据生成多边形导航数据，可以保存为Json文件(Unity计算的NavMesh数据不能直接使用，需要处理下重复顶点 共边的情况)
    2 服务器跟客户端使用同一种寻路算法，导航数据也一致

   难点：比较难实现动态障碍的规避 
   需要扩展已有算法
   {
      1 Recast网上资料很多的，就是对接dll，调用c++来计算路径。如果我来讲这个的话我想全面理解recast源码，然后讲它的算法
      2.对于大部分mmo而言，特别是手机端的，寻路都是不做人物与人物之间碰撞的，这对服务器来说开销过大不划算，而且人多的情况下相互拥挤推拉，影响操作体验。
      人与环境的碰撞有两种思路，一是改变地形（适合长时间的固定变化）二是给物体加上碰撞器，在行进的过程中不停地去做射线检测看是否可以前进通过，当遇到阻挡时尝试左右调整个速度方向避开障碍物。
      Unity中的导航就是这两种实现的。你可以去看官网的那个Unity导航的课程。

      3.对于服务器来说，其实就是把unity的功能搬过去，这两种方式都可以做。但要做的话开发内容会比较多且十分麻烦，
      第一种需要重新计算地图的连通数据结构，通常是只对变化区域受影响的多边形进行动态调整重新划分（多边形分割算法），重新划分并更新边界连接信息，然后对地图当前受影响的导航角色重新计算寻路点。
      第二种就是根据物体的运动信息，速度与方向，根据与当前变更地形之间的距离进行速度方向调整，避开障碍物。可以搜索RVO算法做扩展了解。
   }
}

第二种：利用现有的recastnavigation开源库
{
   1 客户端导出模型数据obj文件给recastnavigation使用
   2 在recastnavigation中生成网格数据
   3 封装Unity调用recastnavigation的动态库，在Unity中使用
   4 客户端服务器都使用recastnavigation来进行寻路，支持动态寻路的
}



]==]

--[==[

raycastNavgation 只是提供的算法 导航网格数据不一定只能它自己生成 也可以其他工具生成
   {
      导航的三种形式：
         1.SoloMesh模式是静态的导航网格，即对场景build-一次之后，将导航网格缓存起来供寻路使用，后续不再允许场景的导航信息发生变化。
         2.TileMesh也是静态的导航网格，只是与SoloMesh相比它按tile来处理地图。
         3.TempObstacles模式可以支持向场景中动态添加或移除预设形状的阻挡物，导航网格也会随之更新（不过只支持添加阻挡物，而不支持添加新的可行走区域)在处理动态阻挡时，
         由于单个阻挡对地图的影响区域是有限的，所以会采用将地图切割成多个固定大小的tile,以tile为单位进行网格的生成。这样在添加或移除阻挡时，只需要处理与阻挡相交的tile,而不需要处理整个地图。
   }

总结一下：
1.recast:负责根据提供的模型生成导航网格
2.Detour::利用导航网格做寻路导航
3.detourcrowd:提供了群体寻路的功能
4.demo:一个很完善的功能演示

]==]

--[==[
   NavMesh  https://www.lmlphp.com/user/58145/article/item/1667160/
   {
      NavMeshAgent的碰撞是指只处理NavMeshAgent之间的碰撞，不能跟Collider进行碰撞
      NavMesh导航时会忽略Physics系统本身的碰撞，也就是说NavMeshAgent在移动的过程中不会被Collider阻挡，而是会直接走过去（但是OnTriggerEnter等触发功能正常）。
         
   
      动态碰撞的功能对很多游戏都是一个基本的需求，而根据NavMesh提供的接口，唯一可以实现阻挡功能的只有NavMeshObstacle，而NavMeshObstacle只有一种形状：圆柱体，而且up方向固定，不能调整为侧向。总结起来就是以下几点：

   　　（1）导航网格的行走/碰撞区域只能预烘焙；

   　　（2）动态碰撞体只能通过挂载NavMeshObstacle组件来实现；

   　　（3）碰撞体的形状只有一种——圆柱体，严格来说就是圆形，而且是正圆还不能是椭圆


      https://www.cnblogs.com/yaukey/p/navmesh_data_export.html
      https://www.cnblogs.com/yaukey/p/3585226.html
      https://www.cnblogs.com/yaukey/p/recastnavigation_unity_optimize_visible_result.html

      https://www.cnblogs.com/yaukey/p/rts_unit_traverse_size_based_path_finding.html


      1.unity 的寻路系统确实是基于 RecastNavigation 编写的，但是 Detour 系统已经被深度改写了，（后来发现作者的 twitter 简介是 unity 的 ai 程序员）；
      2.这个问题确实由于“细长”三角形引起的，NavMesh 系统当遇到特别宽或者三边长度比特别大的三角形时会出问题

      解决方法是：改变A*寻路结果中节点的放置处理，NavMesh 系统里默认使用多边形边的中点来作为路径通过的点，
      如果想要好看的结果，应该使用距离最近的点，即你有 A, B 两个点在一条线段 L 的两边，那么这个点应该是线段 AB 和线段 L 的交点


      只使用NavMeshAgent的碰撞 不使用用寻路 ： 设置updatePosition为false 或者 speed为0
   }


   
   实际使用也发现NavMeshAgent之间的碰撞是很精确的，于是我想了个取巧的办法，给地表再铺一层跟AStar插件的寻路面积一样大的NavMesh，但是没有任何的阻挡（理解为一片纯蓝色），
   同时给寻路单位添加一个NavMeshAgent，然而这个东西只用它的碰撞，而不使用寻路（没有阻挡，任何寻路都是两点直线，可以考虑几乎无开销），这样最后实际的碰撞效果才算达到我们满意的效果。

   NavMeshAngent 只使用它的碰撞不用它的寻路
]==]


