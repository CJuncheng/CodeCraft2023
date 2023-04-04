#include "read_until_ok.h"

ReadUntilOK::ReadUntilOK()
{}

ReadUntilOK::~ReadUntilOK()
{}

bool ReadUntilOK::getMapData(void)
{
    char line[128];
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            fprintf(stderr, "Read map data successed!\n");
            puts("OK"); fflush(stdout);
            return true;
        }
        vector<char> lineMap;
        for(int i = 0; i < 100; ++i) lineMap.emplace_back(line[i]);
        mapData.emplace_back(lineMap);
    }
    return false;
}

bool ReadUntilOK::getInFrameData(void)
{
    char line[256];
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            inFrameDataProc();
            return true;
        }
        string tmpStr;
        istringstream iss(line);
        vector<string> tmpVec;
        while(iss>>tmpStr) tmpVec.emplace_back(tmpStr);
        inFrameData.emplace_back(tmpVec);
    }
    return false;
}

void ReadUntilOK::inFrameDataProc(void)
{
    const int nRows = inFrameData.size();
    for(int i = 0; i != nRows; ++i){
        int nCols = inFrameData[i].size();
        switch(nCols){
            case 2: {
                in_frame_ID = stoi(inFrameData[i][0]), cur_money = stoi(inFrameData[i][1]);
                break;
            }
            case 1: {
                bench_num = stoi(inFrameData[i][0]);
                break;
            }
            case 6: {
                Bench bench_data(stoi(inFrameData[i][0]), stof(inFrameData[i][1]), stof(inFrameData[i][2]),
                                 stoi(inFrameData[i][3]), stoi(inFrameData[i][4]), stoi(inFrameData[i][5]));
                inFrameBenchData.emplace_back(bench_data);
                break;
            }
            case 10: {
                Robot robot_data(stoi(inFrameData[i][0]), stoi(inFrameData[i][1]), 
                stof(inFrameData[i][2]), stof(inFrameData[i][3]), stof(inFrameData[i][4]), stof(inFrameData[i][5]),
                stof(inFrameData[i][6]), stof(inFrameData[i][7]), stof(inFrameData[i][8]), stof(inFrameData[i][9]));
                inFrameRobotData.emplace_back(robot_data);
                break;
            }
            default: break;
        }

    }

}

