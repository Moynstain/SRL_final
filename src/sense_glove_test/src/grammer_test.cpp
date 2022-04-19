//
// Created by harlab on 2022/4/18.
//
#include "iostream"
#include "vector"
#include "cstdlib"
using namespace std;
#define MaxSize 10
// const MaxSize = 10;
vector<vector<double>> matmul_cpp_func(vector<vector<double>> a, vector<vector<double>> b){
    float temp, temp_sum = 0;
    vector<vector<double>> test;
    vector<double> test_temp;
    for (int i = 0; i < a.size(); i++){
        for (int j = 0; j < b.size(); j++){
            for (int k = 0; k < a[0].size(); k++){
                temp_sum+= a[i][k] * b[k][i];
            }
            test_temp.push_back(temp_sum);
        }
        test.push_back(test_temp);
        test_temp.erase(test_temp.begin(), test_temp.end());
    }
    return test;
}

double temp1[MaxSize] = {0};//存的时候由于已知矩阵的尺寸 因此可将矩阵存入一维数组(逐行存) 以避免调用函数时的二维数组传值
double temp2[MaxSize] = {0};
double result[MaxSize] = {0};//两矩阵相乘的输出同样为一维 还需后续处理
// int row_1, col_1, row_2, col_2;
// int i, j;

void matmul_test(int row_a, int col_a, double *temp_a, int row_b, int col_b, double *temp_b, double *matmul_result)
{
    int times = col_a;//times=col_1=row_2 为确定result某元素时进行的乘法(加法)次数
    int row_result, col_result;
    int num = 0;
    //num为一维数组形式的result的索引
    //row_result col_result为二维数组形式的result的索引
    for (row_result = 1; row_result <= row_a; row_result++)
    {
        for (col_result = 1; col_result <= col_b; col_result++)
        {
            result[num] = 0;
            for (int i = 0; i <= times-1; i++)
            {
                //对于m*n大小数组 如需访问其i*j元素 其对应的一维坐标为(i-1)*n+j
                //temp1 为row_1*col_1 需访问其col_row*(i+1)的元素 对应一维坐标为(col_result-1)*col_1+(i+1)
                //temp2 为row_2*col_2 需访问其(i+1)*col_result的元素 对应一维坐标为i*col_2+col_result
                //result[num] += temp1[(row_result - 1)*col_1 + i] * temp2[i*row_2 + col_result-(i+1)];
                result[num] += temp_a[(row_result - 1)*col_a + i] * temp_b[i*col_b + col_result -1 ];
            }
            num++;
        }
    }
}


vector<vector<double>> mat_mul_ult(vector<vector<double>> a, vector<vector<double>> b){
    int row_a = a.size();
    int col_a = a[0].size();
    int row_b = b.size();
    int col_b = b[0].size();
    if (row_a != col_a || row_b != col_b){
        for (int i = 0, )
    }



}


int main(){
    vector<vector<double>> Rot_x = { {1,0,0,0},
                                    {0,0,-1,0},
                                    {0,1,0,0},
                                    {0,0,0,1},
                                    {0,0,0,1} };

    vector<vector<double>> Trans = { {1,0,0,0},
                                    {0,1,0,0},
                                    {0,0,1,0},
                                    {-98, 44.9, 17.25, 1} };

    vector<vector<double>> a = {{1, 2},
                               {3, 4}};
    vector<vector<double>> b = {{1, 2},
                               {3, 4}};
   // for(int i = 0;)
    cout << Rot_x[1].size() << endl;
    cout << Rot_x.size() << endl;
    //auto result = matmul_cpp_func(Rot_x, Trans);
    // auto result = matmul_cpp_func(a, b);
    auto result = multil(Rot_x, Trans);
    for (int i = 0; i < result.size(); i++){
        for (int j = 0; j < result[0].size(); j++){
            cout << result[i][j] << "  ";
        }
        cout << endl;
        cout << endl;
    }
    auto result2 = matmul_cpp_func(a, b);
    for (int i = 0; i < result2.size(); i++){
        for (int j = 0; j < result2[0].size(); j++){
            cout << result2[i][j] << "  ";
        }
        cout << endl;
        cout << endl;
    }

    return 0;
}

