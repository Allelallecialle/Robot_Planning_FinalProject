#include "collision_checker.hpp"
#include <cmath>
#include <algorithm>

static double pointToSegmentDistance(double px, double py,
                                     double ax, double ay,
                                     double bx, double by){
    double dx = bx - ax;
    double dy = by - ay;
    double len2 = dx*dx + dy*dy;

    if(len2 < 1e-12)
        return std::sqrt((px-ax)*(px-ax) + (py-ay)*(py-ay));

    double t = ((px-ax)*dx + (py-ay)*dy) / len2;
    t = std::max(0.0, std::min(1.0, t));

    double cx = ax + t*dx;
    double cy = ay + t*dy;

    return std::sqrt((px-cx)*(px-cx) + (py-cy)*(py-cy));
}

static double distanceToPolygonBoundary(double x, double y,
                                        const geometry_msgs::Polygon& polygon){
    double best = 1e18;
    int n = polygon.points.size();

    for(int i=0;i<n;i++){
        int j = (i+1)%n;
        best = std::min(best,
                        pointToSegmentDistance(x, y,
                                               polygon.points[i].x, polygon.points[i].y,
                                               polygon.points[j].x, polygon.points[j].y));
    }
    return best;
}

// Disk of radius clearance collides when centre is within obs.radius + clearance.
bool pointInsideCylinder(double x, double y, const WorldModel::Obstacle& obs, double clearance){
    double cx = obs.polygon.points[0].x;
    double cy = obs.polygon.points[0].y;

    double dx = x - cx;
    double dy = y - cy;

    double r = obs.radius + clearance;

    return dx*dx + dy*dy <= r*r;
}

// True if inside CCW box or within clearance of an edge.
bool pointInsideBox(double x, double y, const WorldModel::Obstacle& obs, double clearance){
    const auto& pts = obs.polygon.points;
    int n = pts.size();

    bool inside = true;
    for(int i=0; i<n; i++){
        int j = (i+1)%n;

        double ex = pts[j].x - pts[i].x;
        double ey = pts[j].y - pts[i].y;

        double px = x - pts[i].x;
        double py = y - pts[i].y;

        double cross = ex*py - ey*px;

        if(cross < 0){
            inside = false;
            break;
        }
    }

    if(inside)
        return true;

    if(clearance <= 0.0)
        return false;

    return distanceToPolygonBoundary(x, y, obs.polygon) <= clearance;
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
    const double clearance = world.clearance;

    if(!isInsideMap(x, y, world.borders)){
        return false;
    }
    if(clearance > 0.0 && distanceToPolygonBoundary(x, y, world.borders) < clearance){
        return false;
    }

    for(const auto& obs : world.obstacles){
        if(obs.radius > 0){
            if(pointInsideCylinder(x,y,obs,clearance)){
                return false;
            }
        }
        else{
            if(pointInsideBox(x,y,obs,clearance)){
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
    double resolution = 0.05;
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
