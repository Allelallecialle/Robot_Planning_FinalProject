#include "task/reference_generation.hpp"

#include "collision_checker.hpp"

std::vector<comb::RefSample> generateReferenceFromGraphPath(const RoadmapGraph& roadmap, const std::vector<int>& graph_path, double start_yaw, const WorldModel& world){
    std::vector<comb::RefSample> reference;
    std::vector<comb::Vec2> waypoints;

    for(int nodeId : graph_path){
        comb::Vec2 p;
        p.x = roadmap.nodes[nodeId].x;
        p.y = roadmap.nodes[nodeId].y;
        waypoints.push_back(p);
    }

    if(waypoints.size() < 2)
        return reference;

    const double k_max = 1.0 / 0.35;   // max curvature (min turning radius 0.35 m)
    const double v_max = 0.3;
    const double dt = 0.01;

    // The roadmap edges are collision-checked only as STRAIGHT segments, but the
    // reference the robot actually follows is a chain of bounded-curvature Dubins
    // arcs that bulge outside those segments and can clip an obstacle. So after
    // stitching each arc we RE-CHECK its discretized samples against the world;
    // if a leg collides we insert the midpoint of the offending (collision-free)
    // straight leg and re-optimise the headings, which shrinks the bulging arc.
    // Same guard as comb::planTour on the combinatorial side.
    std::vector<comb::Vec2> pts = waypoints;
    const int max_subdiv = 12;

    for(int iter = 0; iter <= max_subdiv; iter++){
        std::vector<double> headings =
            comb::optimizeHeadings(pts, start_yaw, 0.0, k_max, 72);

        reference.clear();
        double tOffset = 0.0;
        int bad_leg = -1;

        for(size_t i=0;i+1<pts.size();i++){
            comb::DubinsCurve curve =
                comb::dubinsShortestPath(
                    pts[i].x,
                    pts[i].y,
                    headings[i],
                    pts[i+1].x,
                    pts[i+1].y,
                    headings[i+1],
                    k_max);

            if(!curve.valid){
                bad_leg = (int)i;
                break;
            }

            std::vector<comb::RefSample> leg;
            comb::appendDiscretizedDubins(curve, v_max, dt, tOffset, leg);

            bool clear = true;
            for(const auto& s : leg){
                if(!isPointValid(s.x, s.y, world)){
                    clear = false;
                    break;
                }
            }
            if(!clear){
                bad_leg = (int)i;
                break;
            }

            for(size_t s = (i==0 ? 0 : 1); s < leg.size(); s++)
                reference.push_back(leg[s]);

            if(!reference.empty())
                tOffset = reference.back().t + dt;
        }

        if(bad_leg < 0)
            break;   // the whole stitched trajectory is collision-free

        comb::Vec2 mid;
        mid.x = (pts[bad_leg].x + pts[bad_leg+1].x) / 2.0;
        mid.y = (pts[bad_leg].y + pts[bad_leg+1].y) / 2.0;
        pts.insert(pts.begin() + bad_leg + 1, mid);
    }

    return reference;
}
