//
// Created by harlab on 2022/4/18.
//

/// 本文档为 ~/script/2_final_trans_all.py的c++版本

#include "iostream"
#include "vector"
#include "/opt/ros/noetic/include/std_msgs/String.h"
#include "/opt/ros/noetic/include/std_msgs/UInt16MultiArray.h"
using namespace std;

vector<vector<double>> matmul_cpp_func(vector<vector<double>> a, vector<vector<double>> b){
    float temp_sum = 0;
    vector<vector<double>> result;
    vector<double> result_temp;
    for (int i = 0; i < a.size(); i++){
        for (int j = 0; j < b.size(); j++){
            for (int k = 0; k < a[0].size(); k++){
                temp_sum+= a[i][k] * b[k][i];
            }
            result_temp.push_back(temp_sum);
        }
        result.push_back(result_temp);
        result_temp.erase(result_temp.begin(), result_temp.end());
    }
    return result;
}

// double x, double y, double depth
vector<double> tracker_to_T265(double x, double y, double depth){

    vector<double> tracker_point[] = {};
    tracker_point->push_back(x);
    tracker_point->push_back(y);
    tracker_point->push_back(depth);
    tracker_point->push_back(1);

    vector<vector<double>> Rot_x = { {1,0,0,0},
                                    {0,0,-1,0},
                                    {0,1,0,0},
                                    {0,0,0,1} };
    Rot_x[1].size();
    vector<vector<double>> Trans = { {1,0,0,0},
                                    {0,1,0,0},
                                    {0,0,1,0},
                                    {-98, 44.9, 17.25, 1} };
    // Rot_x = np.matrix([[1,0,0,0], [0,0,-1,0], [0,1,0,0], [0,0,0,1]]);
    // Trans = np.matrix([[1,0,0,0], [0,1,0,0], [0,0,1,0], [-98, 44.9, 17.25, 1]]);
    auto T_matrix = matmul_cpp_func(Rot_x, Trans);
    // auto T265_point = matmul_cpp_func(tracker_point, T_matrix); // np.matmul(tracker_point, T_matrix);
    vector<double> T265_gaze_point;
    return T265_gaze_point;
}

