#ifndef GEODESIC
#define GEODESIC

#include <vector>
#include <queue>
#include <limits>

template <class ScalarType>
bool Dijkstra(const std::vector<std::vector<ScalarType> > &Weight,
              const std::vector<std::vector<int> >  &graph,
              int &Source,std::vector<std::vector<ScalarType> > &Dist)
{
    assert(Weight.size()==graph.size());
    int numVertices = graph.size();

    // Create a vector to store the distances from the source vertex
    std::vector<int> distance(numVertices, std::numeric_limits<int>::max());

    // Create a priority queue to store vertices with their distances
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int> >, std::greater<std::pair<int, int>>> pq;

//    //Set the distance of the source vertex to 0 and add it to the priority queue
//    distance[source] = 0;
//        pq.push(make_pair(0, source));

//        while (!pq.empty()) {
//            int u = pq.top().second;
//            pq.pop();

//            // Visit each adjacent vertex of u
//            for (const Edge& edge : graph[u]) {
//                int v = edge.destination;
//                int weight = edge.weight;

//                // If a shorter path is found, update the distance and enqueue the vertex
//                if (distance[u] + weight < distance[v]) {
//                    distance[v] = distance[u] + weight;
//                    pq.push(make_pair(distance[v], v));
//                }
//            }
//        }
}

#endif
