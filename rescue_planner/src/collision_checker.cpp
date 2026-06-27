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

bool isInsideMap(double x,double y,const geometry_msgs::Polygon& polygon){
    bool inside = false;
    int n = polygon.points.size();

    for(int i = 0, j = n - 1; i < n; j = i++){
        double xi = polygon.points[i].x;
        double yi = polygon.points[i].y;
        double xj = polygon.points[j].x;
        double yj = polygon.points[j].y;

        bool intersect = ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        if(intersect){
            inside = !inside;
        }
    }
    return inside;
}

bool isPointValid(double x, double y, const WorldModel& world){
    if(!isInsideMap(x, y, world.borders)){
        return false;
    }

    for(const auto& obs : world.obstacles){
        if(obs.radius > 0){
            if(pointInsideCylinder(x,y,obs)){
                return false;
            }
        }
        else{
            if(pointInsideBox(x,y,obs)){
                return false;
            }
        }
    }
    return true;
}

bool isSegmentValid(double x1,double y1,double x2,double y2,const WorldModel& world){
    double dx = x2 - x1;
    double dy = y2 - y1;

    double length = std::sqrt(dx*dx + dy*dy);
    double resolution = 0.05;   // 5 cm
    int steps = std::max(1, (int)(length / resolution));

    for(int i = 0; i <= steps; i++){
        double t = (double)i / steps;

        double x = x1 + t * dx;
        double y = y1 + t * dy;

        if(!isPointValid(x, y, world)){
            return false;
        }
    }

    return true;
}