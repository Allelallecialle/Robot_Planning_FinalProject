// Generic graph algorithms shared by every planner. They all produce a RoadmapGraph and hand it here
#include "task/roadmap_algorithms.hpp"

#include <limits>
#include <queue>
#include <algorithm>

namespace roadmap
{

//   Compute the shortest roadmap distance from a POI to every other node, obtaining
//   the all pairs distance matrix that the Orienteering solver needs
// dist[i] = shortest distance from source to i, infinity if unreachable
// prev[i] = predecessor of i on the shortest path, -1 if unreachable or if i == source
void dijkstra(const RoadmapGraph& graph, int source, std::vector<double>& dist, std::vector<int>& prev){
    const int N = graph.nodes.size();

    dist.assign(N,std::numeric_limits<double>::infinity());
    prev.assign(N,-1);

    // Min-heap of (tentative_distance, node), popped in increasing distance order
    // It always finalizes the currently closest unvisited node next
    using Item = std::pair<double,int>;

    std::priority_queue<
        Item,
        std::vector<Item>,
        std::greater<Item>> pq;

    dist[source]=0.0;

    pq.push({0.0,source});

    while(!pq.empty()){
        auto current=pq.top();
        pq.pop();

        double d=current.first;
        int u=current.second;

        // Already found a shorter way to u since this entry was pushed.
        // This graph's edges are symmetric so a
        // node can be pushed multiple times with different distances
        if(d>dist[u])
            continue;

        // Relax every neighbor of u: if going through u is cheaper than the
        // best known way to reach it so far, update and repush
        for(const auto& edge:graph.adjacency[u]){
            double nd=d+edge.cost;

            if(nd<dist[edge.to]){
                dist[edge.to]=nd;
                prev[edge.to]=u;

                pq.push({nd,edge.to});
            }
        }
    }
}

// Recover the actual sequence of graph node indices on the shortest path.
// Dijkstra only computes distances + parents, not explicit paths
std::vector<int> reconstructPath(int source, int target, const std::vector<int>& prev){
    std::vector<int> path;
    int current=target;

    while(current!=-1){
        path.push_back(current);

        if(current==source)
            break;

        current=prev[current];
    }

    std::reverse(path.begin(),path.end());

    // If the reconstructed path is partial reject it
    if(path.empty() || path.front()!=source)
        return {};

    return path;
}

// Runs Dijkstra once from each POI poi[i] and reads off the
// distance to every other poi[j], producing the
// |poi|x|poi| distance matrix the Orienteering solver operates on.
std::vector<std::vector<double>>
computeDistanceMatrix(const RoadmapGraph& graph, const std::vector<int>& poi){
    const int P=poi.size();

    std::vector<std::vector<double>> D(
        P,
        std::vector<double>(P));

    for(int i=0;i<P;i++){
        std::vector<double> dist;
        std::vector<int> prev;

        dijkstra(graph,poi[i],dist,prev);

        for(int j=0;j<P;j++)
            D[i][j]=dist[poi[j]];
    }

    return D;
}

}