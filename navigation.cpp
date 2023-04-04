#include "navigation.h"

Navigation::Navigation()
{}

Navigation::~Navigation()
{}

float Navigation::cmptrSellProfit(int goods_type, float time_vaule_coef, float collision_value_coef)
{
    return goods_price_tbl[goods_type-1][1]*time_vaule_coef*collision_value_coef - goods_price_tbl[goods_type-1][0];
}

vector<int> Navigation::cmptrBenchIDByGoodsType(int goods_type) // 计算某一物品类型可卖给的所有工作台编号[0, K)。 sell stage
{
    //const int ben_num = inFrameBenchData.size();  // K<=50
    vector<int> bench_ID_vec;
    set<int> bench_type_st = goods_type_match_bench_type[goods_type-1]; // goods_type 映射的工作台类型
    for(int bench_ID = 0; bench_ID < bench_num; ++bench_ID){
        int bench_type = inFrameBenchData[bench_ID].bench_type;
        if(bench_type_st.count(bench_type)) 
            bench_ID_vec.emplace_back(bench_ID);
    }
    return bench_ID_vec;
}

vector<int> Navigation::cmptrDemandOfGoodsType(void)   // 购买物品时，考虑每种物品的需求量
{
    vector<int> demand_of_goods_type = {0, 0, 0, 0, 0, 0, 4}; // 购买物品时，考虑每种物品的需求, 物品类型 1-7
    for(int bench_ID = 0; bench_ID < bench_num; ++bench_ID){
        int bench_type_ = inFrameBenchData[bench_ID].bench_type;
        int material_cell_state_ = inFrameBenchData[bench_ID].material_cell_state;
        if(bench_type_ <=3 ) continue;
        switch (bench_type_)
        {
            case 4: {
                if(!((material_cell_state_ >> 1)&0x1)) ++demand_of_goods_type[0];
                if(!((material_cell_state_ >> 2)&0x1)) ++demand_of_goods_type[1];
                break;
            }
            case 5: {
                if(!((material_cell_state_ >> 1)&0x1)) ++demand_of_goods_type[0];
                if(!((material_cell_state_ >> 3)&0x1)) ++demand_of_goods_type[2];
                break;
            }
            case 6: {
                if(!((material_cell_state_ >> 2)&0x1)) ++demand_of_goods_type[1];
                if(!((material_cell_state_ >> 3)&0x1)) ++demand_of_goods_type[2];
                break;
            }
            case 7: {
                if(!((material_cell_state_ >> 4)&0x1)) ++demand_of_goods_type[3];
                if(!((material_cell_state_ >> 5)&0x1)) ++demand_of_goods_type[4];
                if(!((material_cell_state_ >> 6)&0x1)) ++demand_of_goods_type[5];
                break;
            }
            case 8: {
                if(!((material_cell_state_ >> 7)&0x1)) ++demand_of_goods_type[6];
                break;
            }
            case 9: {
                ++demand_of_goods_type[0];
                ++demand_of_goods_type[1];
                ++demand_of_goods_type[2];
                ++demand_of_goods_type[3];
                ++demand_of_goods_type[4];
                ++demand_of_goods_type[5];
                ++demand_of_goods_type[6];
                break;
            }
            default: break;
        }
    }
    // 考虑所有机器人目前持有的物品
    
    for (int robot_ID = 0; robot_ID < 4; ++robot_ID) {
        switch (inFrameRobotData[robot_ID].goods_type) {
        case 1:
            --demand_of_goods_type[0];
            break;
        case 2:
            --demand_of_goods_type[1];
            break;
        case 3:
            --demand_of_goods_type[2];
            break;
        case 4:
            --demand_of_goods_type[3];
            break;
        case 5:
            --demand_of_goods_type[4];
            break;
        case 6:
            --demand_of_goods_type[5];
            break;
        default:
            break;
        }
    }
    return demand_of_goods_type;
}

vector<vector<int>> Navigation::cmptrBenchIDByBenchType(void) // 计算各种工作台类型的 ID
{
    vector<vector<int>> bench_ID_by_bench_type(7);
    for(int bench_ID = 0; bench_ID < bench_num; ++bench_ID){
        int bench_type = inFrameBenchData[bench_ID].bench_type;
        if(bench_type == 8 || bench_type == 9) continue;
        bench_ID_by_bench_type[bench_type-1].emplace_back(bench_ID);
    }
    return bench_ID_by_bench_type;
}

int Navigation::getBenchTypePriority(int bench_type){ // 考虑 buy 东西时工作台优先级：生产物品工作台，依次 buy 7， 6， 5， 4， 。。。
    int priority = -1;
    switch(bench_type){
        case 1: case 2 : case 3: priority = 0.4; break;
        case 4: case 5: case 6: priority = 0.3; break;
        case 7: priority = 0; break;
        default: break;
    }
    return priority;
}

int Navigation::getBenchTypePriority2(int bench_type){ // 考虑 sell 东西时工作台优先级：消耗物品工作台，依次sell， 4， 5.。。。。
    int priority = -1;
    switch(bench_type){
        case 4: case 5: case 6: case 7: case 8: priority = 0; break;
        case 9: priority = 1.0; break;
        default: break;
    }
    return priority;
}

void Navigation::taskPlan(void)
{
    const int robot_num = inFrameRobotData.size();
    if(robot_num != 4) { //perror("The Robot data frame is error!\n"); 
        return; 
    }

    vector<int> demand_of_goods_type = cmptrDemandOfGoodsType(); // 购买物品时，考虑每种物品的需求, 物品类型 1-7
    //vector<vector<int>> bench_ID_by_bench_type = cmptrBenchIDByBenchType(); // 计算各种工作台类型的 ID

    for(int robot_ID = 0; robot_ID < robot_num; ++robot_ID){
        int goods_type = inFrameRobotData[robot_ID].goods_type;
        if(!goods_type){ //不携带物品 goods_type = 0, buy stage
            /**
             * 考虑是否在某一工作台附近， continue
             */
            int bench_ID = inFrameRobotData[robot_ID].bench_ID;
            if( bench_ID != -1){
                if(inFrameBenchData[bench_ID].product_cell_state && demand_of_goods_type[inFrameBenchData[bench_ID].bench_type - 1]){ 
                    sell_buy_flag[robot_ID] = -1; // buy
                    inFrameBenchData[bench_ID].product_cell_state = 0;
                    --demand_of_goods_type[inFrameBenchData[bench_ID].bench_type - 1];
                    continue; 
                }
            }

            vector<pair<double, int>> score;
            for(int bench_ID = 0; bench_ID < bench_num; ++bench_ID){
                int bench_type = inFrameBenchData[bench_ID].bench_type;
                if(bench_type == 8 || bench_type == 9 || demand_of_goods_type[bench_type-1]<=0) continue;
                Bench bench = inFrameBenchData[bench_ID];
                pair<double, int> score_tmp = {0.0, bench_ID};
                
                if(bench.product_cell_state){ //物品格满
                   
                    // 2.0
                    //if(demand_of_goods_type[bench_type-1]>0){ //工作台 bench_ID 生产的物品卖得出去
                        //score_tmp.first += 0;
                        score_tmp.first += 0.5*getBenchTypePriority(inFrameBenchData[bench_ID].bench_type);
                        //score_tmp.first += 0.2*1.0/demand_of_goods_type[goods_type_];
                        score_tmp.first += 0.5*(abs(inFrameRobotData[robot_ID].x - inFrameBenchData[bench_ID].x) + abs(inFrameRobotData[robot_ID].y - inFrameBenchData[bench_ID].y))/100.0;
                        
                        //--demand_of_goods_type[goods_type_];

                    //} else { //工作台 bench_ID 生产的物品很大概率卖不出去
                        //score_tmp.first += 3;
                        //score_tmp.first += inFrameBenchData[bench_ID].remain_product_time/1000.0;
                    //}
                        
                    /*
                    // 1.0
                    vector<int> bench_ID_vec = cmptrBenchIDByGoodsType(goods_type_);
                    // 判断买的东西能否卖出去
                    size_t flag = 0;
                    for(; flag < bench_ID_vec.size(); ++flag)
                        if(!((inFrameBenchData[bench_ID_vec[flag]].material_cell_state >> goods_type_ )& 0x1)) break;  
                
                    if(flag==bench_ID_vec.size()){ //工作台 bench_ID 生产的物品很大概率卖不出去
                        score_tmp.first += 3;
                        score_tmp.first += inFrameBenchData[bench_ID].remain_product_time/1000.0;
                    } else { //工作台 bench_ID 生产的物品卖得出去
                        score_tmp.first += 0;
                        //score_tmp.first += 0.65*getBenchTypePriority(inFrameBenchData[bench_ID].bench_type)/3.0;
                        score_tmp.first += 0.5*(abs(inFrameRobotData[robot_ID].x - inFrameBenchData[bench_ID].x) + abs(inFrameRobotData[robot_ID].y - inFrameBenchData[bench_ID].y))/100.0;
                    }
                    */
                    
                } else { // 物品格空
                    score_tmp.first += 3;
                    score_tmp.first += inFrameBenchData[bench_ID].remain_product_time/1000.0;
                }
                
                //if(coef_vec_buy[0]) { //考虑产品格状态
                //    score_tmp.first += coef_vec_buy[0]*(bench.product_cell_state ? 0 : 6); // 产品格有东西打 0 分
                //}
                //if(coef_vec_buy[1]){ // 考虑距离
                //    score_tmp.first += coef_vec_buy[1]*(abs(inFrameRobotData[robot_ID].x - inFrameBenchData[bench_ID].x) + 
                //    abs(inFrameRobotData[robot_ID].y - inFrameBenchData[bench_ID].y))/100.0;
                //}
            
                score.emplace_back(score_tmp);
            }
            

            sort(score.begin(), score.end());
            //避免四个小车购买目标一样
            set<int> st(task_bench_ID.begin(), task_bench_ID.end());
            auto it =  score.begin();
            while(it!=score.end()&&st.count(it->second)) ++it;
            if(it==score.end()){ //发生概率小
                fprintf(stderr, "[ERR_INFO] Robot %d can not buy goods to bench!\n", robot_ID);
                continue;
            }
            
            //if(it->first >=6) continue;  // 调节贪心策略
            task_bench_ID[robot_ID] = it->second;      
            
        } else { //携带物品 [1, 7], sell stage
            /**
             * 考虑是否在某一工作台附近， continue
             */
            int bench_ID = inFrameRobotData[robot_ID].bench_ID;
            
            if( bench_ID != -1){
                fprintf(stderr, "[ERR_INFO] Robot %d carry goods %d near bench %d!\n", robot_ID, goods_type, bench_ID);
                if(!((inFrameBenchData[bench_ID].material_cell_state >> goods_type)&0x1)){
                    sell_buy_flag[robot_ID] = 1; // sell
                    fprintf(stderr, "[ERR_INFO] Robot %d sell goods %d near bench %d!\n", robot_ID, goods_type, bench_ID);

                    inFrameBenchData[bench_ID].material_cell_state |= (1 << goods_type);
                    //--demand_of_goods_type[inFrameBenchData[bench_ID].bench_type - 1];
                    if(inFrameBenchData[bench_ID].product_cell_state && demand_of_goods_type[inFrameBenchData[bench_ID].bench_type - 1]){//buy
                        sell_buy_flag[robot_ID] = 2;
                        inFrameBenchData[bench_ID].product_cell_state = 0;
                        --demand_of_goods_type[inFrameBenchData[bench_ID].bench_type - 1];
                    }
                    continue; 
                }
            }
            
            vector<int> bench_ID_vec = cmptrBenchIDByGoodsType(goods_type);
            vector<pair<double, int>> score;  // 得分越低优先级越高
            for(size_t i = 0; i < bench_ID_vec.size(); ++i){ //这里顺序已经带有工作台类型(bench_type) 优先级
                Bench bench = inFrameBenchData[bench_ID_vec[i]];
                pair<double, int> score_tmp = {0.0, bench_ID_vec[i]};

                // 考虑材料格状态
                score_tmp.first += 2*((bench.material_cell_state >> goods_type) & 0x1);

                // 考虑距离
                score_tmp.first += 0.6*((abs(inFrameRobotData[robot_ID].x - bench.x) + 
                    abs(inFrameRobotData[robot_ID].y - bench.y))/100.0);
                
                // 考虑剩余生产时间 ---> 为了买了能立即卖
                if(bench.bench_type <= 8)
                    score_tmp.first += 0.416*(bench.remain_product_time/1000.0);
                else 
                    score_tmp.first += 0.416*0.99;
                
                // 考虑工作台类型
                score_tmp.first += 1.0*getBenchTypePriority2(bench.bench_type);
                score.emplace_back(score_tmp);
            }
            sort(score.begin(), score.end());
            
            
            //避免携带相同物品的小车目标一样
            auto it =  score.begin();
            for(int i = 0; i < robot_ID; ++i){
                if((task_bench_ID[i]==it->second) && (inFrameRobotData[i].goods_type == inFrameRobotData[robot_ID].goods_type))
                    ++it;
            } 
            if(it==score.end()){ // 基本不会发生
                fprintf(stderr, "[ERR_INFO] Robot %d can not find bench to sell goods!\n", robot_ID); 
                continue;
            }
            /*
            if(it->first>1) { // 得分大于一 ----> 没有找到材料格空闲的工作台
                fprintf(stderr, "[ERR_INFO] Robot %d can not find bench whose material cell state is empty!\n", robot_ID);
                destroy_flag[robot_ID] = 1;
                //continue; // 找不到材料格空闲的工作台， 携带物品的小车不作动作
            }
            */
            task_bench_ID[robot_ID] = it->second; 
        }
    }
}

vector<int> Navigation::cmptrRobotPriority(void) // 在避免碰撞时，检测小车的优先级
{
    vector<int> robot_priority(4);
    for(size_t robot_ID = 0; robot_ID < 4; ++robot_ID)
        if(inFrameRobotData[robot_ID].goods_type)
            robot_priority[robot_ID] = cmptrSellProfit(inFrameRobotData[robot_ID].goods_type, inFrameRobotData[robot_ID].time_vaule_coef, inFrameRobotData[robot_ID].collision_value_coef);
    return robot_priority;
}

vector<vector<int>> Navigation::collisionDect(void) // 在机器人动态行驶中进行动态碰撞检测
{
    /*
    vector<vector<double>> coord(4, vector<double>(2));
    for(size_t robot_ID = 0; robot_ID < 4; ++robot_ID) {
        coord[robot_ID][0] = inFrameRobotData[robot_ID].x;
        coord[robot_ID][1] = inFrameRobotData[robot_ID].y;
    }
    */
    
    // 考虑机器人 0 的避障
    const double dist_thold = 1;
    const double theta_thold = 1;
    vector<vector<int>> adj(4, vector<int>(4, -1));
    for(size_t i = 0; i < 4; ++i)
        for(size_t robot_ID = i+1; robot_ID < 4; ++robot_ID) {
            double dist = sqrt(pow(inFrameRobotData[i].x-inFrameRobotData[robot_ID].x, 2) + pow(inFrameRobotData[i].y-inFrameRobotData[robot_ID].y, 2));
            if(dist > dist_thold) continue;

            // 计算夹角
            double theta1 = inFrameRobotData[i].angle - atan2(inFrameRobotData[robot_ID].y - inFrameRobotData[i].y, inFrameRobotData[robot_ID].x - inFrameRobotData[i].x);
            double theta2 = inFrameRobotData[robot_ID].angle - atan2(inFrameRobotData[i].y - inFrameRobotData[robot_ID].y, inFrameRobotData[i].x - inFrameRobotData[robot_ID].x);
            
            if((abs(theta1) <= theta_thold || abs(theta1) >= 2*M_PI - theta_thold) && (abs(theta2) <= theta_thold || abs(theta2) >= 2*M_PI - theta_thold)){
                adj[i][robot_ID] = adj [robot_ID][i] = robot_ID;
                fprintf(stderr, "打印碰撞邻接矩阵：\n");

            }

            /* 一类

            if((theta1>0 && theta1 < theta_thold) || (theta1 < (theta_thold - 2*M_PI))){ // 连线左侧
            }
            if((theta2<0 && theta2 > -theta_thold) || (theta2 > (2*M_PI - theta_thold))){ // 连线右侧
            }
            */
        }
/*
    fprintf(stderr, "打印碰撞邻接矩阵：\n");
    for(int i = 0; i < 4; ++i){
        for(int j = 0; j < 4; ++j){
            fprintf(stderr, "%d ", adj[i][j]);
        }
        fprintf(stderr, "\n");
    }
    */
    return adj;

}

void Navigation::motionControl(void)
{   
    
    //if(in_frame_ID > 8650) return;
    pFrameID(in_frame_ID);

    //cmptrRobotPriority();
    // 碰撞检测
    //vector<int> robot_priority = cmptrRobotPriority();
    //vector<vector<int>> adj = collisionDect();
    //vector<bool> vst(4, false); //判断机器人是否做出避障动作

    for(size_t robot_ID = 0; robot_ID < 4; ++robot_ID){
        if(sell_buy_flag[robot_ID] >= 1) { // sell
            if(sell_buy_flag[robot_ID]==1){
                sell(robot_ID);
                forward(robot_ID,2.5); // 3.0
                rotate(robot_ID, 0.8); // 3.0
                //rotate(robot_ID, 3);
            } else { // sell then buy
                sell(robot_ID);
                buy(robot_ID);
                forward(robot_ID, 2.5);
                rotate(robot_ID, 0.8);
                //rotate(robot_ID, 3);
            }
            continue;
        }
        if(sell_buy_flag[robot_ID] == -1) { // buy
            forward(robot_ID, 2.5);
            rotate(robot_ID, 0.8);
            buy(robot_ID);
            continue;
        }
/*
        for(int i = 0; i < 4; ++i){
            if(vst[robot_ID]) continue;
            if(adj[robot_ID][i]!=-1&& !vst[i]){
                
                if(!inFrameRobotData[robot_ID].goods_type) {      
                    rotate(robot_ID, -M_PI); 
                    forward(robot_ID, 6);
                    vst[robot_ID] = true;
                } else {
                    if(!inFrameRobotData[i].goods_type) {  
                        rotate(i, -M_PI); 
                        forward(i, 6);
                        vst[i] = true;
                    } else { //均携带物品
                        if(robot_priority[robot_ID] > robot_priority[i]) {
                            rotate(i, -M_PI); 
                            forward(i, 6);
                            vst[i] = true;
                        } else {
                            rotate(robot_ID, -M_PI); 
                            forward(robot_ID, 6);
                            vst[robot_ID] = true;
                        }
                    }
                }
            }
        }
        if(vst[robot_ID]) continue;
 */
        if(task_bench_ID[robot_ID]==-1) {
            //fprintf(stderr, "[ERR_INFO] Robot %d can not find bench!", robot_ID);
            continue;
        }

        const double x1=inFrameRobotData[robot_ID].x, y1=inFrameRobotData[robot_ID].y, 
                    x2=inFrameBenchData[task_bench_ID[robot_ID]].x, y2 = inFrameBenchData[task_bench_ID[robot_ID]].y;
        double dist = sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
        double theta1 = atan2(y2 - y1, x2 - x1); // 计算以机器人为起点到目标站点的连线与x轴的夹角 (-pi, pi] 
        //if(theta1<0) theta1 += 2*M_PI;
        double theta2 = inFrameRobotData[robot_ID].angle;
        //if(theta2<0) theta2 += 2*M_PI;
        double delta_theta = theta2 - theta1;

        double thold1 = 0.5;//0.18; // 角度阈值
        /** 
         * 参数1： thold1 = 0.18; 行驶 rotate： 0.15； 买卖 forward：3.2
         * 参数2： thold1 = 0.5; 行驶 rotate： 0.15； 买卖 forward：3.2
         */ 

        if((abs(delta_theta) <= thold1) || (abs(delta_theta) >= (2*M_PI - thold1))){
            if(dist > 0.435){
                if((delta_theta>0&&delta_theta < thold1) || (delta_theta < (thold1 - 2*M_PI))){ // 连线左侧
                    rotate(robot_ID, -0.15);
                } else { //连线右侧
                    rotate(robot_ID, 0.15); 
                }
                forward(robot_ID, 6);
            } else { // 到达
                if((delta_theta>0&&delta_theta < thold1) || (delta_theta < (thold1 - 2*M_PI))){ // 连线左侧
                    rotate(robot_ID, -0.15); 
                } else { //连线右侧
                    rotate(robot_ID, 0.15); 
                }
                forward(robot_ID, 3.2);
            }
        } else if((abs(delta_theta) > thold1 && abs(delta_theta) <= M_PI_2) || ((abs(delta_theta) < (2*M_PI - thold1)) && abs(delta_theta)>=1.5*M_PI)){
            if(dist > 1.5){
                if((delta_theta > thold1 && delta_theta <= M_PI_2) || (delta_theta > (thold1 - 2*M_PI)&& delta_theta <= -1.5*M_PI)){ // 连线左侧
                    rotate(robot_ID, -M_PI); 
                } else { //连线右侧
                    rotate(robot_ID, M_PI); 
                }
                forward(robot_ID, 6);
            } else {
                if((delta_theta > thold1 && delta_theta <= M_PI_2) || (delta_theta > (thold1 - 2*M_PI)&& delta_theta <= -1.5*M_PI)){ // 连线左侧
                    rotate(robot_ID, -M_PI); 
                } else { //连线右侧
                    rotate(robot_ID, M_PI); 
                }
                forward(robot_ID, 3.2);
            }

        } else {
            if(dist < 1.5 && abs(delta_theta) >= M_PI- thold1 && abs(delta_theta) >= M_PI+thold1) {
                    if((delta_theta >= M_PI- thold1 && delta_theta < M_PI) ||(delta_theta >= -M_PI-thold1 && delta_theta < -M_PI)){ // 连线反向右侧
                        rotate(robot_ID, 0.15); 
                    } else { // 连线反向左侧
                        rotate(robot_ID, -0.15); 
                    }
                    forward(robot_ID, -2);
            } else {
                if((delta_theta > M_PI_2 && delta_theta <= M_PI) || (delta_theta > -1.5*M_PI && delta_theta <= -M_PI)){
                    rotate(robot_ID, -M_PI); 
                    forward(robot_ID, -2);
                } else {
                    rotate(robot_ID, M_PI); 
                    forward(robot_ID, -2);
                }
            }
        }
    }
    puts("OK"); fflush(stdout);
}
