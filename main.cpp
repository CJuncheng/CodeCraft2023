#include "navigation.h"
//#include <bits/stdc++.h>
//#include <unistd.h>

int main(int argc, char* argv[]) {
    // sleep(20);
    Navigation obj;
    obj.getMapData();
    //int eofFlag;
    while (scanf("%d %d", &obj.in_frame_ID, &obj.cur_money) != EOF){
    //while (1){
        //fprintf(stderr, "in_frame_ID: %d\n", obj.in_frame_ID);
        //if(obj.in_frame_ID >= 8750) continue;
        obj.getInFrameData();
        //if(obj.in_frame_ID==100) break;
        obj.taskPlan();
        obj.motionControl();
        obj.reset();
    }
    return 0;
}




/*
int main() {
    readUntilOK();
    puts("OK");
    fflush(stdout);
    int frameID;
    while (scanf("%d", &frameID) != EOF) {  // 处理非初始化的每一帧数据
        readUntilOK();
        printf("%d\n", frameID);
        int lineSpeed = 3;
        double angleSpeed = 1.5;
        for(int robotId = 0; robotId < 4; robotId++){
            printf("forward %d %d\n", robotId, lineSpeed);
            printf("rotate %d %f\n", robotId, angleSpeed);
        }
        printf("OK\n", frameID);
        fflush(stdout);
    }
}
*/