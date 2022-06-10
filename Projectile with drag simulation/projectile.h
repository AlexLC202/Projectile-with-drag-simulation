#pragma once

#include <math.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>

using namespace std;

class projectile
{
public:
    projectile(double Cd, double R, double p, double m);

    void createMap();
    tuple<double, double, double> findShotsFromMap(double distance);
    double getPartialDer(double angle, double velocity, double distance);

    bool simulate(double thetaInit, double vInit, double distance, bool print, double kkDt);
    void findPathSolutions(double distance);
    void calcAllPairs();
    double calcDistance(double theta, double v);

    void printGoal(double distance);

    double calcAx();
    double calcAy();
private:
    const double pi = 3.14159;

    const double kDt = 0.0001;
    const double GOAL_HEIGHT = 2.6416 - 0.5334;
    const double MAX_VELOCITY = 50.0;
    const double MAX_ANGLE = 61.5;
    const double MIN_ANGLE = 41.5;
    

    const double g = 9.8;

    const double Cd_;
    const double R_;
    const double p_;
    const double m_;

    const double k = Cd_ * R_ * R_ * pi * p_ / (2 * m_);

    double vx_, vy_, v_, theta_;

    double x_, y_;

    std::map<double, std::pair<double, double>> distanceMap_, distanceMap2_;
    std::map<std::pair<double, double>, double> angVelMap_;

};