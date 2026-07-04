#include "task/reference_generation.hpp"

std::vector<comb::RefSample> generateReferenceFromGraphPath(const RoadmapGraph& roadmap, const std::vector<int>& graph_path, double start_yaw){
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

    std::vector<double> headings =
        comb::optimizeHeadings(waypoints, start_yaw, 0.0, 1.0 / 0.35, 72);

    double tOffset = 0.0;

    for(size_t i=0;i+1<waypoints.size();i++){
        comb::DubinsCurve curve =
            comb::dubinsShortestPath(
                waypoints[i].x,
                waypoints[i].y,
                headings[i],
                waypoints[i+1].x,
                waypoints[i+1].y,
                headings[i+1],
                1.0/0.35);

        if(!curve.valid)
            continue;

        std::vector<comb::RefSample> leg;

        comb::appendDiscretizedDubins(
            curve,
            0.3,
            0.01,
            tOffset,
            leg);

        if(reference.empty())
        {
            reference.insert(
                reference.end(),
                leg.begin(),
                leg.end());
        }
        else
        {
            reference.insert(
                reference.end(),
                leg.begin()+1,
                leg.end());
        }

        if(!reference.empty())
            tOffset = reference.back().t + 0.01;
    }

    return reference;
}