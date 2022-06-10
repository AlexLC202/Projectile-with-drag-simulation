#include "projectile.h"

projectile::projectile(double Cd, double R, double p, double m) : Cd_(Cd), R_(R), p_(p), m_(m)
{
    ofstream outstream("projectile.csv");
    outstream.close();

    createMap();
}

void projectile::createMap()
{
    ifstream infile("distance.csv");
    string data;
    double distance, angle, velocity;



    bool valid;
    int c1, c2;

    //int failsafe = 0;
    //int duplicates = 0;
    while(getline(infile, data))
    {
        valid = true;

        c1 = data.find(", ");
        if(c1 != string::npos)
        {
            distance = stod(data.substr(0, c1));

            c2 = data.find(", ", c1 + 1);
            if(c2 != string::npos)
            {
                angle = stod(data.substr(c1 + 2, c2));
                velocity = stod(data.substr(c2 + 2));
            }
            else
            {
                valid = false;
            }
        }
        else
        {
            valid = false;
        }

        if(valid)
        {
            pair<double, double> distancePair(angle, velocity);
            pair<double, pair<double, double>> distancePoint(distance, distancePair);

            if(distanceMap_.find(distance) == distanceMap_.end())
            {
                distanceMap_.insert(distancePoint);
            }
            else
            {
                if(distanceMap2_.find(distance) != distanceMap2_.end())
                {
                    cout << "pain" << endl;
                }
                distanceMap2_.insert(distancePoint);
                //cout << distance << endl;
                //++duplicates;
            }

            


            pair<double, double> angVelPair(angle, velocity);
            pair<pair<double, double>, double> angVelPoint(angVelPair, distance);
            angVelMap_.insert(angVelPoint);


        }
        
        //failsafe++;

    }

    //cout << duplicates << endl;
    //cout << failsafe << endl;
}

tuple<double, double, double> projectile::findShotsFromMap(double distance)
{
    double lowest = 1000; //TODO find number and stuff yeah
    pair<double, double> bestShot;
    bool hasShot = false;

    for(double i = -0.025; i < 0.025; i += 0.00001)
    {
        double val = (int)((distance + i) * 100000) / 100000.0;
        //cout << distance + i << endl;

        if(distanceMap_.find(val) != distanceMap_.end())
        {
            //cout << val << endl;
            pair<double, double> pair1 = distanceMap_.find(val)->second;

            double partDer = getPartialDer(pair1.first, pair1.second, val);
            if (lowest > partDer)
            {
                lowest = partDer;
                bestShot = pair<double, double>(pair1.first, pair1.second);
                hasShot = true;
            }

            //simulate(pair.first, pair.second, distance, true, 10);
        }
        else if(distanceMap2_.find(val) != distanceMap2_.end())
        {
            pair<double, double> pair2 = distanceMap2_.find(val)->second;

            double partDer = getPartialDer(pair2.first, pair2.second, val);
            if (lowest > partDer)
            {
                lowest = partDer;
                bestShot = pair<double, double>(pair2.first, pair2.second);
                hasShot = true;
            }

            //simulate(pair.first, pair.second, distance, true, 10);
        }
    }

    //cout << lowest << endl;
    //cout << bestShot.first << ", " << bestShot.second << endl;
    //simulate(bestShot.first, bestShot.second, distance, false, 10);

    if(hasShot)
    {
        return tuple<double, double, double>(bestShot.first, bestShot.second, lowest);
    }
    else
    {
        return tuple<double, double, double>(0, 0, 12345);
    }
    

    
    //pair<double, double> pair2 = map_.find(7.12236)->second;
    //simulate(pair2.first, pair2.second, distance, true);
}


double projectile::getPartialDer(double angle, double velocity, double distance)
{
    double dAngUp, dAngDown, dVelUp, dVelDown;

    double angUp = (int)((angle + 1) * 100) / 100.0;
    double angDown = (int)((angle - 1) * 100) / 100.0;
    double velUp = (int)((velocity + 0.05) * 100) / 100.0;
    double velDown = (int)((velocity - 0.05) * 100) / 100.0;

    if (angVelMap_.find(pair<double, double>(angUp, velocity)) != angVelMap_.end())
    {
        dAngUp = angVelMap_.find(pair<double, double>(angUp, velocity))->second;
    }
    else { dAngUp = -1;  }

    if (angVelMap_.find(pair<double, double>(angDown, velocity)) != angVelMap_.end())
    {
        dAngDown = angVelMap_.find(pair<double, double>(angDown, velocity))->second;
    }
    else { dAngDown = -1; }

    if (angVelMap_.find(pair<double, double>(angle, velUp)) != angVelMap_.end())
    {
        dVelUp = angVelMap_.find(pair<double, double>(angle, velUp))->second;
    }
    else { dVelUp = -1; }

    if (angVelMap_.find(pair<double, double>(angle, velDown)) != angVelMap_.end())
    {
        dVelDown = angVelMap_.find(pair<double, double>(angle, velDown))->second;
    }
    else { dVelDown = -1; }

    /*double dAngUp = angVelMap_.find(pair<double, double>(angUp, velocity))->second;
    double dAngDown = angVelMap_.find(pair<double, double>(angDown, velocity))->second;

    double dVelUp = angVelMap_.find(pair<double, double>(angle, velUp))->second;
    double dVelDown = angVelMap_.find(pair<double, double>(angle, velDown))->second;*/
    
    double angDer, velDer;

    if(dAngUp == -1 && dAngDown != -1)
    {
        angDer = abs(distance - dAngDown) * 2;
    }
    else if(dAngUp != -1 && dAngDown == -1)
    {
        angDer = abs(distance - dAngUp) * 2;
    }
    else
    {
        angDer = abs(dAngUp - dAngDown);
    }


    if (dVelUp == -1 && dVelDown != -1)
    {
        velDer = abs(distance - dVelDown) * 2;
    }
    else if (dVelUp != -1 && dVelDown == -1)
    {
        velDer = abs(distance - dVelUp) * 2;
    }
    else
    {
        velDer = abs(dVelUp - dVelDown);
    }


    return  angDer + velDer; //TODO test weights
}



bool projectile::simulate(double thetaInit, double vInit, double distance, bool print, double kkDt)
{
    ofstream outstream("projectile.csv", ios_base::app);
    

    x_ = 0;
    y_ = 0;
    v_ = vInit;
    theta_ = thetaInit * pi / 180;
    vx_ = v_ * cos(theta_);
    vy_ = v_ * sin(theta_);

    int failsafe = 0;
    while(true)
    {
        vx_ += calcAx() * (kDt * kkDt);
        vy_ += calcAy() * (kDt * kkDt);
        v_ = sqrt(vx_ * vx_ + vy_ * vy_);
        theta_ = atan2(vy_, vx_);

        x_ += vx_ * (kDt * kkDt);
        y_ += vy_ * (kDt * kkDt);

        if(print)
        {
            outstream << x_ << ", " << y_ << endl;
        }
        else
        {
            outstream << "0, 0, " << x_ << ", " << y_ << endl;
        }
        

        if(x_ > distance + 0.127 && x_ < distance + 1.093 && y_ < GOAL_HEIGHT && vy_ < 0)
        {
            //cout << "passed" << endl;
            return true;
        }

        if (x_ < distance + 0.05 && x_ > distance - 0.05 && y_ < GOAL_HEIGHT)
        {
            //cout << "hit the wall" << endl;
            return false;
        }

        if (x_ > distance + 1.22 && y_ > GOAL_HEIGHT + 0.127)
        {
            //cout << "too far" << endl;
            return false;
        }

        if(y_ < 0)
        {
            //cout << "too low" << endl;
            return false;
        }

        if (failsafe > 100000)
        {
            //cout << "failsafe" << endl;
            return false;
        }


        failsafe++;
    }

    outstream.close();
}

void projectile::findPathSolutions(double distance)
{
    //ofstream outstream;
    //outstream.open("paths.csv");
    int i = 0;

    double initAngle = atan2(GOAL_HEIGHT, distance) * 180 / pi;
    initAngle = (initAngle > MIN_ANGLE) ? initAngle : MIN_ANGLE;
    
    vector<pair<double, double>> points;

    for(double angle = initAngle; angle < MAX_ANGLE; angle += 0.5)
    {
        //optimize later
        for(double velocity = 0; velocity < MAX_VELOCITY; velocity += 0.2)
        {
            //cout << angle << ", " << velocity;
            if(simulate(angle, velocity, distance, false, 1))
            {
                ++i;
                pair<double, double> point(angle, velocity);
                points.push_back(point);
            }
        }
    }

    cout << i << endl;

    for (int i = 0; i < points.size(); ++i)
    {
        //cout << points[i].first << ", " << points[i].second << endl;
        simulate(points[i].first, points[i].second, distance, true, 1);
    }

    printGoal(distance);

    //outstream.close();

}

void projectile::calcAllPairs()
{
    ofstream outstream("distance.csv");
    double distance;
    
    for(double angle = MIN_ANGLE; angle < MAX_ANGLE; angle += 0.05)
    {
        for (double velocity = 0; velocity < MAX_VELOCITY; velocity += 0.05)
        {
            distance = calcDistance(angle, velocity);

            if(distance != -1 && distance < 8.5)
            {
                outstream << distance << ", " << angle << ", " << velocity << endl;
            }

        }
    }

    outstream.close();
}

double projectile::calcDistance(double theta, double v)
{
    x_ = 0;
    y_ = 0;
    v_ = v;
    theta_ = theta * pi / 180;
    vx_ = v_ * cos(theta_);
    vy_ = v_ * sin(theta_);

    bool overGoal = false;

    while (true)
    {
        vx_ += calcAx() * kDt;
        vy_ += calcAy() * kDt;
        v_ = sqrt(vx_ * vx_ + vy_ * vy_);
        theta_ = atan2(vy_, vx_);

        x_ += vx_ * kDt;
        y_ += vy_ * kDt;

        if(y_ > GOAL_HEIGHT)
        {
            overGoal = true;
        }

        if(!overGoal && vy_ < 0)
        {
            return -1;
        }
        else if(overGoal && y_ < GOAL_HEIGHT && vy_ < 0)
        {
            return x_ - 0.73025; //9.5 inches is 0.2413, 2 feet is 0.6096, 28.5 inches is 0.73025
        }

        if(y_ < 0)
        {
            return -1;
        }
        
    }
}

void projectile::printGoal(double distance)
{
    ofstream outstream("projectile.csv", ios_base::app);

    for(double i = distance; i < distance + 1.22; i += 0.05)
    {
        outstream << "0, 0, " << i << ", " << GOAL_HEIGHT << endl;
    }
}

double projectile::calcAx()
{
    return k * -cos(theta_) * v_ * v_;
}

double projectile::calcAy()
{
    return k * -sin(theta_) * v_ * v_ - g;
}

int main()
{
    projectile projectile(0.507, 0.12065, 1.225, 0.27);
    cout << "start" << endl;

    //projectile.calcAllPairs();

    //projectile.findShotsFromMap(7.21); //7.21

    ofstream outstream("best.csv");
    tuple<double, double, double> bestShot, prevBestShot;

    for (double i = 1.5; i < 9; i += 0.0001)
    {
        bestShot = projectile.findShotsFromMap(i);

        if (bestShot != prevBestShot && get<2>(bestShot) != 12345)
        {
            cout << i << endl;
            outstream << i << ", " << get<0>(bestShot) << ", " << get<1>(bestShot) << ", " << get<2>(bestShot) << endl;
            prevBestShot = bestShot;
        }
    }
    outstream.close();
    

}