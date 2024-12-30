/**
 * @file CosineCurve.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <cmath>

class CosineCurve{

public:
    CosineCurve(){};

    double getPos(double t){
        if (t < t0_){
            return x0_;
        } else if (t > t1_){
            return x1_;
        } else {
            double t_normalized = (t - t0_) / (t1_ - t0_);
            double w = 0.5 - 0.5 * cos(M_PI * t_normalized);
            return x0_ * (1 - w) + x1_ * w;
        }
    };

    double getVel(double t){
        if (t < t0_ || t > t1_){
            return 0.0;
        } else {
            double t_normalized = (t - t0_) / (t1_ - t0_);
            return 0.5 * M_PI * (x1_ - x0_) * sin(M_PI * t_normalized) / (t1_ - t0_);
        }
    };
    
    void reset(double x0, double x1, double t0, double t1){
        x0_ = x0;
        x1_ = x1;
        t0_ = t0;
        t1_ = t1;
    };

    double getStartTime(){
        return t0_;
    };

    double getEndTime(){
        return t1_;
    };

private:
    double x0_;
    double x1_;
    double t0_;
    double t1_;
};