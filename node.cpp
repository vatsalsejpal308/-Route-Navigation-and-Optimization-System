#include<string>
using namespace std;

class GraphNode {
public: 
    int id;
    string name;
    GraphNode(int id , string name) {
        this -> id = id;
        this -> name = name;
    }
};