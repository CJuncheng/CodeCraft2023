#ifndef NAVIGATION_H
#define NAVIGATION_H
#include "read_until_ok.h"

#include<set>
#include <algorithm>
#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath> 
/**
 * #define M_PI      3.14159265358979323846
 * #define M_PI_2    1.57079632679489661923
 * #define M_PI_4    0.78539816339744830962
 */ 

class Navigation : public ReadUntilOK
{

public:
    Navigation();
    ~Navigation();
    
    void taskPlan(void);
    void motionControl(void);
    void reset(void){
        inFrameBenchData.clear();
        inFrameRobotData.clear();
        inFrameData.clear();
        sell_buy_flag = {0, 0, 0, 0};
        destroy_flag = {0, 0, 0, 0};
        task_bench_ID = vector<int>(4, -1);
    }
    
private:
    const vector<vector<int>> goods_price_tbl = {{3000, 6000}, {4400, 7600}, {5800, 9200}, 
                        {15400, 22500}, {17200, 25000}, {19200, 27500}, {76000, 105000}};
    float cmptrSellProfit(int goods_type, float time_vaule_coef, float collision_value_coef);

    const vector<set<int>> goods_type_match_bench_type = {{4, 5, 9}, {4, 6, 9}, {5, 6, 9}, {7, 9}, {7, 9}, {7, 9}, {8, 9}};
    vector<int> cmptrBenchIDByGoodsType(int goods_type);   // 计算某一物品类型对应的所有工作台编号[0, K)。 sell stage
    int getBenchTypePriority(int bench_type); // 考虑 buy 东西时工作台优先级：
    int getBenchTypePriority2(int bench_type); // 考虑 sell 东西时工作台优先级
    vector<int> cmptrDemandOfGoodsType(void);   // 购买物品时，考虑每种物品的需求量
    vector<vector<int>> cmptrBenchIDByBenchType(void); // 计算各种工作台类型的 ID
    vector<int> cmptrRobotPriority(void); // 在避免碰撞时，检测小车的优先级
    vector<vector<int>> collisionDect(void); // 在机器人动态行驶中进行动态碰撞检测

    
    /* 机器人售卖货物，选择某个工作台时， 考虑各个因素的系数：材料格状态系数， 距离系数， 剩余生产时间系数，工作台类型系数， 携带物品利润系数
     *  0: 不考虑
     *  (0, ): 值越低权重越高
     */
    //const vector<double> coef_vec_sell = {1, 0.6, 0.4, 0, 0}; 

    /* 机器人购买货物，选择某个工作台时， 考虑各个因素的系数：产品格状态系数，距离系数，剩余生产时间系数，工作台类型系数， 
     *  0: 不考虑
     *  (0, ): 值越低权重越高
     */
    //const vector<double> coef_vec_buy = {1, 1, 0, 0, 0}; 
    vector<int> sell_buy_flag = {0, 0, 0, 0}; // 默认0，不卖不卖。 1 ：sell。 -1 buy。 2: sell then buy。
    vector<int> destroy_flag = {0, 0, 0, 0}; // 默认0，不销毁； 1： 销毁
    vector<int> task_bench_ID = vector<int>(4, -1); // 4个机器人对应的四个目标工作台的编号 [0, K)

    //动作组
    void pFrameID(int frame_Id){printf("%d\n", frame_Id);}
    void forward(int robot_Id, double speed) { printf("forward %d %f\n", robot_Id, speed);}//[-2, 6]
    void rotate(int robot_Id, double angle_speed) { printf("rotate %d %f\n", robot_Id, angle_speed);}//[-pi, pi]
    void buy(int robot_Id) { printf("buy %d\n", robot_Id); }
    void sell(int robot_Id) { printf("sell %d\n", robot_Id); }
    void destroy(int robot_Id) { printf("destroy %d\n", robot_Id);}
};

#endif