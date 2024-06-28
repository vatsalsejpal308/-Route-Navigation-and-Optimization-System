#include "node.cpp"
#include<unordered_map>
#include<map>
#include<vector>
#include<queue>
#include<algorithm>
#include<iostream>
using namespace std;

class Graph {
private:
    unordered_map<int , GraphNode*> nodes;
    unordered_map<int , unordered_map<int , pair<double , double>>> adjacencyList; // node1 -> node2 -> (distance , speed limit)
    map<pair<int , int> , double> averageSpeed;
public:
    string getName(int id) {
        if(nodes.find(id) != nodes.end()) {
            return nodes[id] -> name;
        }
        return "";
    }

    void addNode(int id , string name) {
        if(nodes.find(id) == nodes.end()) {
            GraphNode* node = new GraphNode(id , name);
            nodes[id] = node;
        }
    }

    void removeNode(int id) {
        //removing from nodes
        if(nodes.find(id) != nodes.end()) {
            GraphNode* gnode = nodes[id];
            nodes.erase(id);
            delete gnode;
        }

        //removing all neighbors of id and those edges from averageSpeed map
        if(adjacencyList.find(id) != adjacencyList.end()) {
            for(auto& data : adjacencyList[id]) {
                if(averageSpeed.find({id , data.first}) != averageSpeed.end()) {
                    averageSpeed.erase({id , data.first});
                }
            }
            adjacencyList.erase(id);
        }

        //removing edges which id is a neighbor of and those edges from averageSpeed
        for(auto& data : adjacencyList) {
            if(data.second.find(id) != data.second.end()) {
                data.second.erase(id);
            }
            if(averageSpeed.find({data.first , id}) != averageSpeed.end()) {
                averageSpeed.erase({data.first , id});
            }
        }
    }

    void addRoute(int id1 , int id2 , double distance , double speedLimit) {
        adjacencyList[id1][id2] = {distance , speedLimit};
    }

    void removeRoute(int id1 , int id2) {
        if(adjacencyList[id1].find(id2) != adjacencyList[id1].end()) {
            adjacencyList[id1].erase(id2);
        }
        if(averageSpeed.find({id1 , id2}) != averageSpeed.end()) {
            averageSpeed.erase({id1 , id2});
        }
    }

    void modifyAverageSpeed(int id1 , int id2 , double avgSpeed) {
        averageSpeed[{id1 , id2}] = avgSpeed;
    }

    pair<vector<int> , double> getFastestPath(int source , int destination) {
        unordered_map<int , double> time;
        unordered_map<int , int> parent;
        priority_queue<pair<double , int> , vector<pair<double , int>> , greater<pair<double , int>>> minheap;
        for(auto& data : nodes) {
            time[data.first] = 1e18;
            parent[data.first] = -1;
        }

        time[source] = 0.0;
        minheap.push({0.0 , source});

        while(!minheap.empty()) {
            int node = minheap.top().second;
            double t = minheap.top().first;
            minheap.pop();

            if(t > time[node]) continue;
            if(node == destination) break;

            for(auto& data : adjacencyList[node]) {
                int neighbor = data.first;
                double speed;
                if(averageSpeed.find({node , neighbor}) == averageSpeed.end()) {
                    speed = data.second.second; // the speed limit for lack of data
                } else {
                    speed = min(averageSpeed[{node , neighbor}] , data.second.second); // cant exceed speed limit
                }
                double route_time = data.second.first / speed; //distance / speed
                double new_time = route_time + t;

                if(new_time < time[neighbor]) {
                    time[neighbor] = new_time;
                    parent[neighbor] = node;
                    minheap.push({new_time , neighbor}); 
                }
            }
        }

        if(time[destination] == 1e18) {
            return {{} , -1.0};
        } else {
            vector<int> path;
            int node = destination;
            while(node != -1) {
                path.push_back(node);
                node = parent[node];
            }

            reverse(path.begin() , path.end());
            return {path , time[destination]};
        }
    }

    pair<vector<int> , double> getShortestPath(int source , int destination) {
        unordered_map<int , double> distance;
        unordered_map<int , int> parent;
        priority_queue<pair<double , int> , vector<pair<double , int>> , greater<pair<double , int>>> minheap;
        for(auto& data : nodes) {
            distance[data.first] = 1e18;
            parent[data.first] = -1;
        }

        distance[source] = 0.0;
        minheap.push({0.0 , source});

        while(!minheap.empty()) {
            int node = minheap.top().second;
            double dist = minheap.top().first;
            minheap.pop();

            if(dist > distance[node]) continue;
            if(node == destination) break;

            for(auto& data : adjacencyList[node]) {
                int neighbor = data.first;
                double weight = data.second.first;
                double new_dist = weight + dist;

                if(new_dist < distance[neighbor]) {
                    distance[neighbor] = new_dist;
                    parent[neighbor] = node;
                    minheap.push({new_dist , neighbor});
                }
            }
        }

        if(distance[destination] == 1e18) {
            return {{} , -1.0};
        } else {
            vector<int> path;
            int node = destination;
            while(node != -1) {
                path.push_back(node);
                node = parent[node];
            }

            reverse(path.begin() , path.end());
            return {path , distance[destination]};
        }
    }
};