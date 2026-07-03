#include "task/roadmap_algorithms.hpp"

#include <limits>
#include <queue>
#include <algorithm>

namespace roadmap
{

void dijkstra(const RoadmapGraph& graph, int source, std::vector<double>& dist, std::vector<int>& prev){
    const int N = graph.nodes.size();

    dist.assign(N,std::numeric_limits<double>::infinity());
    prev.assign(N,-1);

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

        if(d>dist[u])
            continue;

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

    if(path.empty() || path.front()!=source)
        return {};

    return path;
}

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