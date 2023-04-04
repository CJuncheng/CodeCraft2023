#ifndef READ_UNTIL_OK_H
#define READ_UNTIL_OK_H

#include <iostream>
#include <vector>
#include <sstream>
using namespace std;

struct Bench { //定义输入帧中每一行工作台数据类型
    int bench_type;
    double x;
    double y;
    int remain_product_time;
    int material_cell_state;
    int product_cell_state;
    Bench(int arg1, double arg2, double arg3, int arg4, int arg5, int arg6) : 
        bench_type(arg1), x(arg2), y(arg3), remain_product_time(arg4), 
        material_cell_state(arg5), product_cell_state(arg6) {}
};

struct Robot { //定义输入帧中每一行机器人数据类型
    int bench_ID;
    int goods_type;
    double time_vaule_coef;
    double collision_value_coef;
    double angle_speed;
    double line_speed_x;
    double line_speed_y;
    double angle;
    double x;
    double y;
    Robot(int arg1, int arg2, double arg3, double arg4, double arg5, double arg6, double arg7, double arg8, double arg9, double arg10) : 
        bench_ID(arg1),goods_type(arg2), time_vaule_coef(arg3), collision_value_coef(arg4), 
        angle_speed(arg5),line_speed_x(arg6), line_speed_y(arg7), angle(arg8), x(arg9), y(arg10) {}
};

class ReadUntilOK
{
public:
    ReadUntilOK();
    ~ReadUntilOK();
    bool getMapData(void);
    bool getInFrameData(void);
    // void clear(void);
    
    vector<vector<char>> mapData;

    //解析输入帧数据
    vector<vector<string>> inFrameData;
    int in_frame_ID = -1, cur_money=-1, bench_num=-1;
    vector<Bench> inFrameBenchData;
    vector<Robot> inFrameRobotData;
    
private:
    
    void inFrameDataProc(void);
};

#endif
