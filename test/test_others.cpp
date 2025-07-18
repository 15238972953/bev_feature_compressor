#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

// 生成带噪声的线性数据
vector<pair<double, double>> generateData(int n, double slope, double intercept, double noiseLevel) {
    vector<pair<double, double>> data;
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<> noise(0, noiseLevel);

    for (int i = 0; i < n; ++i) {
        double x = static_cast<double>(i) / n * 10.0;
        double y = slope * x + intercept + noise(gen);
        data.emplace_back(x, y);
    }
    return data;
}

// 最小二乘法实现
pair<double, double> leastSquares(const vector<pair<double, double>>& data) {
    int n = data.size();
    MatrixXd X(n, 2);
    VectorXd y(n);

    // 填充设计矩阵X和观测向量y
    for (int i = 0; i < n; ++i) {
        X(i, 0) = data[i].first;  // x值
        X(i, 1) = 1.0;           // 常数项
        y(i) = data[i].second;   // y值
    }

    // 使用QR分解求解最小二乘问题
    VectorXd beta = X.colPivHouseholderQr().solve(y);

    return {beta(0), beta(1)};  // 返回斜率和截距
}

int main() {
    // 真实参数
    double trueSlope = 2.0;
    double trueIntercept = 1.0;
    double noiseLevel = 0.5;
    int n = 100;  // 数据点数量

    // 生成数据
    auto data = generateData(n, trueSlope, trueIntercept, noiseLevel);

    // 使用最小二乘法拟合
    auto [slope, intercept] = leastSquares(data);

    // 输出结果
    cout << "真实模型: y = " << trueSlope << "x + " << trueIntercept << endl;
    cout << "拟合模型: y = " << slope << "x + " << intercept << endl;

    // 计算均方误差
    double mse = 0.0;
    for (const auto& [x, y] : data) {
        double predicted = slope * x + intercept;
        mse += (predicted - y) * (predicted - y);
    }
    mse /= n;
    cout << "均方误差: " << mse << endl;

    return 0;
}