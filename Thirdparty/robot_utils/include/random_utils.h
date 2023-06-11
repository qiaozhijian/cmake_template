/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * author: QIAO Zhijian
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <cmath>
#include <cassert>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <random>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <random>

template<typename T>
Eigen::Matrix<T, 3, 1> sampleMultivariateNormal(const Eigen::Matrix<T, 3, 1> &mean,
                                                             const Eigen::Matrix<T, 3, 3> &cov) {
    // 使用 Cholesky 分解计算协方差矩阵的平方根
    Eigen::LLT<Eigen::Matrix<T, 3, 3>> chol(cov);
    Eigen::Matrix<T, 3, 3> L = chol.matrixL();

    // 生成随机数
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<T> distribution(0.0, 1.0);

    // 生成随机向量
    int n = mean.size();
    Eigen::Matrix<T, 3, 1> x(n);
    for (int i = 0; i < n; ++i) {
        x(i) = distribution(gen);
    }

    // 计算样本向量
    Eigen::Matrix<T, 3, 1> z = mean + L * x;
    return z;
}

template<typename T>
Eigen::Matrix<T, 3, 3> randomRotation(const T &angle_bound) {
    Eigen::Matrix<T, 3, 1> axis = Eigen::Matrix<T, 3, 1>::Random();
    axis.normalize();
    // random [-1, 1]
    T angle = angle_bound * Eigen::internal::random<T>() / 180.0 * M_PI;
    return Eigen::AngleAxis<T>(angle, axis).toRotationMatrix();
}

template<typename T>
Eigen::Matrix<T, 3, 1> randomTranslation(const T &translation_bound) {
    // random [-1, 1] for each element
    return Eigen::Matrix<T, 3, 1>::Random() * translation_bound;
}

template<typename T>
Eigen::Matrix<T, 4, 4> randomTransformation(const T &angle_bound, const T &translation_bound) {
    Eigen::Matrix<T, 4, 4> ans = Eigen::Matrix<T, 4, 4>::Identity();
    ans.template block<3, 3>(0, 0) = randomRotation(angle_bound);
    ans.template block<3, 1>(0, 3) = randomTranslation(translation_bound);
    return ans;
}

template<typename T>
inline T random_normal(T std) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<T> dis(0, std);
    T number = dis(gen);
    return number;
}

template<typename T>
inline T random_uniform(T min, T max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<T> dis(min, max);
    T number = dis(gen);
    return number;
}