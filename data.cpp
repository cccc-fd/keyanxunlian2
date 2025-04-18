#define _USE_MATH_DEFINES

#include "data.h"


// ******************************** 不变配置信息 **********************************
const int nRun = 15;                                    // 运行次数
const int nPts = 3;                                     // 断点个数
const int d = 50;                                       // 计算插值点数量因子
const int nPop = 100;                                   // 粒子数量
const int epochs = 100;                                 // 迭代次数
const std::string f_interp = "cubic";                   // 插值方法：三次样条插值
const int pointNums = 1 + (nPts + 1) * d;               // 插值点数量
const double F = M_PI / 2;                              // 侦察角
const std::vector<int> limits = {0, 200, 0, 200, 50};   // 空间限制：左、右、上、下、高度
const int start[3] = {10, 70, 10};           // 无人机起点：x y 高度


// ******************************** 用例1 **************************************
// 无人机群信息：分别表示：{x, y, h}，即：{x, y, 高度}
//const std::vector<std::vector<int>> uavs_start = {
//        {10, 40, 10},
//        {20, 20, 10},
//        {40, 10, 10},
//        {60, 20, 10}
//};

// 三个数为一个点的坐标，如：{55, 175, 10}是一个点的坐标，代表{x, y, h}，其中h代表高度
// 四个点的顺序为：西南、东南、东北、西北四个角的顺序
//std::unordered_map<std::string, std::vector<std::vector<int>>> goals = {
//        // 点目标
//        {"point_goals",   {}},
//        {"surface_goals", {
//                                  {55, 175, 10, 65, 175, 10, 65, 185, 10, 55, 185, 10},
//                                  {155, 125, 10, 165, 125, 10, 165, 135, 10, 155, 135, 10},
//                                  {130, 170, 10, 190, 170, 10, 190, 180, 10, 130, 180, 10},
//                                  {160, 80, 10, 200, 80, 10, 200, 110, 10, 160, 110, 10}
//                          }}
//};

// 障碍信息
//std::unordered_map<std::string, std::vector<std::vector<int>>> obstacle = {
//        {"radar",    {{130, 100, 0,  20}}},
//        {"gun",      {{120, 40,  0,  30}}},
//        {"missile",  {{70,  150, 0,  15}}},
//        {"mountain", {{50,  40,  15, 10}}},
//        {"no_fly",   {{20,  110, 50, 110, 50, 150, 40, 150, 20, 140}}}
//};
// ******************************** 用例1 end **************************************



// ******************************** 用例2 **************************************
// 无人机群信息：分别表示：{x, y, h}，即：{x, y, 高度}
//const std::vector<std::vector<int>> uavs_start = {
//        {10, 40, 10},
//        {20, 20, 10},
//        {40, 10, 10},
//        {60, 20, 10}
//};

// 三个数为一个点的坐标，如：{55, 175, 10}是一个点的坐标，代表{x, y, h}，其中h代表高度
// 四个点的顺序为：西南、东南、东北、西北四个角的顺序
//std::unordered_map<std::string, std::vector<std::vector<int>>> goals = {
//        // 点目标
//        {"point_goals",   {}},
//        {"surface_goals", {
//                                  {55, 175, 10, 65, 175, 10, 65, 185, 10, 55, 185, 10},
//                                  {155, 125, 10, 165, 125, 10, 165, 135, 10, 155, 135, 10},
//                                  {130, 170, 10, 190, 170, 10, 190, 180, 10, 130, 180, 10},
//                                  {160, 80, 10, 200, 80, 10, 200, 110, 10, 160, 110, 10}
//                          }}
//};

// 障碍信息
//std::unordered_map<std::string, std::vector<std::vector<int>>> obstacle = {
//        {"radar", {{130, 100, 0, 20}}},
//        {"gun", {{120, 40, 0, 30}}},
//        {"missile", {{70, 150, 0, 15}}},
//        {"mountain", {{50, 40, 15, 10}}},
//        {"weather", {{70, 90, 10, 20}, {50, 105, 10, 20}}},
//        {"no_fly", {{20, 110, 50, 110, 50, 150, 40, 150, 20, 140}, {100, 110, 130, 110, 130, 150, 120, 150, 100, 140}}}
//};
// ******************************** 用例2 end **************************************

// ******************************** 用例3 **************************************
// 无人机群信息：分别表示：{x, y, h}，即：{x, y, 高度}
const std::vector<std::vector<int>> uavs_start = {
        {10, 40, 10},
        {20, 20, 10},
        {40, 10, 10},
        {60, 20, 10},
        {50, 5, 10}
};

// 三个数为一个点的坐标，如：{55, 175, 10}是一个点的坐标，代表{x, y, h}，其中h代表高度
// 四个点的顺序为：西南、东南、东北、西北四个角的顺序
std::unordered_map<std::string, std::vector<std::vector<int>>> goals = {
        // 点目标
        {"point_goals",   {}},
        {"surface_goals", {
                                  {55, 175, 10, 65, 175, 10, 65, 185, 10, 55, 185, 10},
                                  {155, 125, 10, 165, 125, 10, 165, 135, 10, 155, 135, 10},
                                  {130, 170, 10, 190, 170, 10, 190, 180, 10, 130, 180, 10},
                                  {150, 150, 10, 210, 150, 10, 210, 160, 10, 150, 160, 10},
                                  {160, 80, 10, 200, 80, 10, 200, 110, 10, 160, 110, 10}
                          }}
};

// 障碍信息
std::unordered_map<std::string, std::vector<std::vector<int>>> obstacle = {
        {"radar", {{130, 100, 0, 20}}},
        {"gun", {{120, 40, 0, 30}}},
        {"missile", {{70, 150, 0, 15}}},
        {"mountain", {{50, 40, 15, 10}}},
        {"weather", {{70, 90, 10, 20}, {50, 105, 10, 20}}},
        {"no_fly", {{20, 110, 50, 110, 50, 150, 40, 150, 20, 140}, {100, 110, 130, 110, 130, 150, 120, 150, 100, 140}}}
};
// ******************************** 用例3 end **************************************
