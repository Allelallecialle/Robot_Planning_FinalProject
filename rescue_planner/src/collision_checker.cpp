#include "collision_checker.hpp"
#include <cmath>

bool pointInsideCylinder(double x, double y, const WorldModel::Obstacle& obs){
    double cx = obs.polygon.points[0].x;
    double cy = obs.polygon.points[0].y;

    double dx = x - cx;
    double dy = y - cy;

    return dx*dx + dy*dy <= obs.radius*obs.radius;
}

bool pointInsideBox(double x, double y, const WorldModel::Obstacle& obs){
    const auto& pts = obs.polygon.points;
    int n = pts.size();

    for(int i=0; i<n; i++){
        int j = (i+1)%n;

        double ex = pts[j].x - pts[i].x;
        double ey = pts[j].y - pts[i].y;

        double px = x - pts[i].x;
        double py = y - pts[i].y;

        double cross = ex*py - ey*px;

        if(cross < 0){
            return false;
        }
    }
    return true;
}

bool isPointValid(double x, double y, const WorldModel& world){
    for(const auto& obs : world.obstacles){
        if(obs.radius > 0){
            if(pointInsideCylinder(x,y,obs))
                return false;
        }
        else{
            if(pointInsideBox(x,y,obs))
                return false;
        }
    }
    return true;
}