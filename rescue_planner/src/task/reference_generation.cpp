#include "task/reference_generation.hpp"

#include <ros/ros.h>
#include <cmath>
#include "collision_checker.hpp"
#include "utils/planning_budget.hpp"
#include "utils/geometry_utils.hpp"

namespace {
double yawFromQuat(const geometry_msgs::Quaternion& q) {
    const double t0 = 2.0 * (q.w * q.z + q.x * q.y);
    const double t1 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(t0, t1);
}
}  // namespace

std::vector<comb::RefSample> generateReferenceFromGraphPath(const RoadmapGraph& roadmap, const std::vector<int>& graph_path, const std::vector<int>& poi_path_nodes, double start_yaw, const WorldModel& world){
    std::vector<comb::RefSample> reference;
    std::vector<comb::Vec2> waypoints;
    
    const double gate_yaw = world.gates.empty() ? 0.0 : yawFromQuat(world.gates[0].orientation);

    for(int nodeId : graph_path){
        comb::Vec2 p;
        p.x = roadmap.nodes[nodeId].x;
        p.y = roadmap.nodes[nodeId].y;
        waypoints.push_back(p);
    }

    if(waypoints.size() < 2)
        return reference;

    // Line-of-sight simplification, per POI-to-POI leg (never across the
    // whole multi-victim tour at once, so a required victim stop can't be
    // shortcut past). This is the same helper Voronoi/Cell Decomposition use
    // in task/tour_builder.cpp -- previously missing here, which let dense
    // PRM/RRT/RRT*/RRG roadmap paths turn every intermediate sampled node
    // into its own Dubins via-point instead of collapsing them into the
    // fewest safe straight legs first.
    if(poi_path_nodes.size() >= 2){
        std::vector<std::size_t> boundary_idx;
        boundary_idx.reserve(poi_path_nodes.size());
        std::size_t search_from = 0;
        for(int poi_node : poi_path_nodes){
            for(std::size_t i = search_from; i < graph_path.size(); i++){
                if(graph_path[i] == poi_node){
                    boundary_idx.push_back(i);
                    search_from = i + 1;
                    break;
                }
            }
        }

        if(boundary_idx.size() == poi_path_nodes.size()){
            std::vector<comb::Vec2> simplified;
            for(std::size_t k = 0; k + 1 < boundary_idx.size(); k++){
                std::vector<comb::Vec2> leg(
                    waypoints.begin() + boundary_idx[k],
                    waypoints.begin() + boundary_idx[k + 1] + 1);

                const std::vector<comb::Vec2> simp = comb::simplifyLineOfSight(
                    leg,
                    [&](const comb::Vec2& a, const comb::Vec2& b){
                        return isSegmentValid(a.x, a.y, b.x, b.y, world);
                    });

                for(std::size_t t = (simplified.empty() ? 0 : 1); t < simp.size(); t++)
                    simplified.push_back(simp[t]);
            }
            waypoints = simplified;
        } else {
            // Should not happen (every POI is a graph_path node by
            // construction), but fail safe rather than silently mis-simplify.
            ROS_WARN("[sampling] could not resolve all POI leg boundaries "
                     "(%lu/%lu found); skipping simplification.",
                     boundary_idx.size(), poi_path_nodes.size());
        }
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
    bool complete = false;

    for(int iter = 0; iter <= max_subdiv; iter++){
        std::vector<double> headings =
            comb::optimizeHeadings(pts, start_yaw, gate_yaw, k_max, 72);

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

        if(bad_leg < 0){
            complete = true;
            break;
        }

        comb::Vec2 mid;
        mid.x = (pts[bad_leg].x + pts[bad_leg+1].x) / 2.0;
        mid.y = (pts[bad_leg].y + pts[bad_leg+1].y) / 2.0;
        pts.insert(pts.begin() + bad_leg + 1, mid);
    }

    // If a leg still clips after all subdivisions, `reference` only holds the
    // arcs BEFORE the offending leg -- a truncated path stopping short of the
    // gate. Reject it (return empty) instead of driving the robot at an obstacle.
    if(!complete)
        reference.clear();

    return reference;
}

SamplingMissionPlan planSamplingMission(RoadmapGraph& roadmap, const WorldModel& world, double start_yaw){
    SamplingMissionPlan plan;

    // v_max / dubins_safety must match computeVictimMission.
    const double v_max = 0.3;
    const double dubins_safety = 0.85;
    double budget = comb::distanceBudget(world.victims_timeout, v_max, dubins_safety);

    // Build POI context once so budget re-solves do not grow the roadmap.
    MissionContext ctx = buildMissionContext(roadmap, world);

    plan.mission = solveMissionWithBudget(ctx, budget);
    if(!plan.mission.feasible)
        return plan;
    plan.reference =
        generateReferenceFromGraphPath(roadmap, plan.mission.graph_path,
                                       plan.mission.poi_path_nodes, start_yaw, world);

    // Tighten graph budget when flyable duration exceeds timeout; re-solve OP only.
    if(world.victims_timeout > 0){
        for(int it = 0;
            it < 8 && plan.mission.feasible && !plan.reference.empty() &&
            plan.reference.back().t > static_cast<double>(world.victims_timeout);
            ++it){
            budget *= (static_cast<double>(world.victims_timeout) /
                       plan.reference.back().t) * 0.98;
            ROS_INFO("[sampling] flyable %.1f s > timeout %d s; retrying with "
                     "budget=%.2f m", plan.reference.back().t,
                     world.victims_timeout, budget);
            plan.mission = solveMissionWithBudget(ctx, budget);
            if(!plan.mission.feasible)
                break;
            plan.reference =
                generateReferenceFromGraphPath(roadmap, plan.mission.graph_path,
                                               plan.mission.poi_path_nodes, start_yaw, world);
        }
    }

    return plan;
}
