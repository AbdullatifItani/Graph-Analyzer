#include <iostream>
#include <unordered_map>
#include <vector>
#include <string>
#include <stack>
#include <queue>
#include <cfloat> 
using namespace std;

struct Node {
    string label;
    unordered_map<Node*, float> neighbors; // Changed to float for weights
    int indegree = 0;
};

struct Graph {
    bool isDirected;
    unordered_map<string, Node*> nodes;

    Graph(bool isDirected) : isDirected(isDirected) {}

    void addNode(string label) {
        if (nodes.count(label)) {
            cout << "Node " << label << " already exists." << endl;
            return;
        }
        Node* newNode = new Node;
        newNode->label = label;
        nodes[label] = newNode;
    }

    void addEdge(string fromLabel, string toLabel, float weight) {
        Node* fromNode = nodes[fromLabel];
        Node* toNode = nodes[toLabel];
        toNode->indegree++;
        fromNode->neighbors[toNode] = weight;
        if (!isDirected) {
            toNode->neighbors[fromNode] = weight;
            fromNode->indegree++;
        }
    }

    void removeNode(string label) {
        if (!nodes.count(label)) {
            cout << "Node " << label << " doesn't exist." << endl;
            return;
        }
        Node* node = nodes[label];
        nodes.erase(label);
        for (auto neighbor : node->neighbors) {
            Node* toNode = neighbor.first;
            toNode->neighbors.erase(node);
            toNode->indegree--;
        }
        node->neighbors.clear();
        delete node;
    }

    void removeEdge(string fromLabel, string toLabel) {
        Node* fromNode = nodes[fromLabel];
        Node* toNode = nodes[toLabel];
        fromNode->neighbors.erase(toNode);
        toNode->indegree--;
        if (!isDirected) {
            toNode->neighbors.erase(fromNode);
            fromNode->indegree--;
        }
    }

    void adjustWeight(string fromLabel, string toLabel, float newWeight) {
        Node* fromNode = nodes[fromLabel];
        Node* toNode = nodes[toLabel];
        fromNode->neighbors[toNode] = newWeight;
        if (!isDirected) {
            toNode->neighbors[fromNode] = newWeight;
        }
    }

    void DFS(string s) {
        unordered_map<string, bool> visited;
        stack<string> stack;
        stack.push(s);
        while (!stack.empty()) {
            string v = stack.top();
            stack.pop();
            if (!visited[v]) {
                visited[v] = true;
                cout << v << " ";
                for (auto it : nodes[v]->neighbors) {
                    stack.push(it.first->label);
                }
            }
        }
    }

    bool isCyclicUtil(unordered_map<string, bool>& visited, unordered_map<string, Node*>& nodes, string curr) {
        if (visited[curr]) return true;
        visited[curr] = true;
        bool f = false;
        for (auto it : nodes[curr]->neighbors) {
            f = isCyclicUtil(visited, nodes, it.first->label);
            if (f) return true;
        }
        visited[curr] = false;
        return false;
    }

    bool isCyclic() {
        unordered_map<string, bool> visited;
        for (auto i : nodes) {
            visited[i.first] = false;
        }
        bool f = false;
        for (auto j : nodes) {
            visited[j.first] = true;
            for (auto k : j.second->neighbors) {
                f = isCyclicUtil(visited, nodes, k.first->label);
                if (f) return true;
            }
            visited[j.first] = false;
        }
        return false;
    }

    void bellmanFord(string source, string dest) {
        unordered_map<string, float> distances;
        unordered_map<string, string> parent;
        for (auto vertex : nodes) {
            distances[vertex.first] = FLT_MAX; // Using FLT_MAX for infinite distance
            parent[vertex.first] = "";
        }
        distances[source] = 0;
        int V = nodes.size();
        for (int i = 0; i < V - 1; i++) {
            for (auto u : nodes) {
                for (auto v : u.second->neighbors) {
                    if (distances[u.first] + v.second < distances[v.first->label]) {
                        distances[v.first->label] = distances[u.first] + v.second;
                        parent[v.first->label] = u.first;
                    }
                }
            }
        }
        bool hasNegativeCycle = false;
        for (auto it : nodes) {
            for (auto j : it.second->neighbors) {
                if (distances[it.first] + j.second < distances[j.first->label]) {
                    hasNegativeCycle = true;
                    break;
                }
            }
        }
        if (hasNegativeCycle) {
            cout << "The graph contains a negative cycle." << endl;
            return;
        }
        vector<string> path;
        string current = dest;
        while (current != source) {
            path.push_back(current);
            current = parent[current];
        }
        path.push_back(source);
        cout << "The shortest path between " << source << " and " << dest << " is ";
        for (int i = path.size() - 1; i >= 0; i--) {
            cout << path[i] << " ";
        }
    }

    void Dijkstra(string source, string dest) {
        unordered_map<string, float> distances;
        unordered_map<string, string> parent;
        for (auto vertex : nodes) {
            distances[vertex.first] = FLT_MAX;
            parent[vertex.first] = "";
        }
        distances[source] = 0;
        queue<string> q;
        q.push(source);
        while (!q.empty()) {
            string u = q.front();
            q.pop();
            for (auto v : nodes[u]->neighbors) {
                if (distances[u] + v.second < distances[v.first->label]) {
                    distances[v.first->label] = distances[u] + v.second;
                    parent[v.first->label] = u;
                    q.push(v.first->label);
                }
            }
        }
        vector<string> path;
        string current = dest;
        while (current != source) {
            path.push_back(current);
            current = parent[current];
        }
        path.push_back(source);
        cout << "The shortest path between " << source << " and " << dest << " is ";
        for (int i = path.size() - 1; i >= 0; i--) {
            cout << path[i] << " ";
        }
    }

    void topologicalSort() {
        vector<string> result;
        unordered_map<string, bool> visited;
        stack<string> stack;
        for (auto it : nodes) {
            if (it.second->indegree == 0) {
                stack.push(it.first);
            }
        }
        while (!stack.empty()) {
            string node = stack.top();
            stack.pop();
            if (!visited[node]) {
                visited[node] = true;
                result.push_back(node);
                for (auto j : nodes[node]->neighbors) {
                    j.first->indegree--;
                    if (j.first->indegree == 0) {
                        stack.push(j.first->label);
                    }
                }
            }
        }
        for (auto i : result) {
            cout << i << " ";
        }
        cout << endl;
    }
};

int main() {
    bool negWeight = false;
    cout << "Is the graph directed or undirected? (Enter 1 for directed, 0 for undirected): ";
    int isDirected;
    cin >> isDirected;
    Graph graph(isDirected);
    while (true) {
        cout << "Enter a command \n0 Exit\n1 Add a node\n2 Remove a node\n3 Add an edge\n4 Adjust the weight of an edge\n5 Remove an edge\n6 DFS Traversal\n7 Detect Cycle\n8 Topological Sort\n9 Find Shortest Path\n>> ";
        int command;
        cin >> command;
        if (command == 0) {
            break;
        } else if (command == 1) {
            cout << "Enter a label for the node to add: ";
            string label;
            cin >> label;
            graph.addNode(label);
        } else if (command == 2) {
            cout << "Enter the label of the node to remove: ";
            string label;
            cin >> label;
            graph.removeNode(label);
            cout << endl;
        } else if (command == 3) {
            cout << "Enter the labels of the two nodes to add an edge between: ";
            string label1, label2;
            cin >> label1 >> label2;
            if (graph.nodes.count(label1) && graph.nodes.count(label2)) {
                cout << "Is the edge weighted? (Enter 1 for yes, 0 for no): ";
                int isWeighted;
                cin >> isWeighted;
                if (isWeighted) {
                    cout << "Enter the weight of the edge: ";
                    float weight;
                    cin >> weight;
                    if (weight < 0) negWeight = true;
                    graph.addEdge(label1, label2, weight);
                } else {
                    graph.addEdge(label1, label2, 0);
                }
            } else {
                cout << "One or both nodes are not found";
            }
            cout << endl;
        } else if (command == 4) {
            cout << "Enter the labels of the two nodes: ";
            string label1, label2;
            cin >> label1 >> label2;
            if (graph.nodes.count(label1) && graph.nodes.count(label2)) {
                graph.removeEdge(label1, label2);
                cout << "Enter the new weight of the edge: ";
                float weight;
                cin >> weight;
                if (weight < 0) negWeight = true;
                graph.adjustWeight(label1, label2, weight);
            } else {
                cout << "One or both nodes are not found";
            }
            cout << endl;
        } else if (command == 5) {
            cout << "Enter the labels of the two nodes to remove the edge between: ";
            string label1, label2;
            cin >> label1 >> label2;
            if (graph.nodes.count(label1) && graph.nodes.count(label2)) {
                graph.removeEdge(label1, label2);
            } else {
                cout << "One or both nodes are not found";
            }
            cout << endl;
        } else if (command == 6) {
            cout << "Enter the label of the node: ";
            string label1;
            cin >> label1;
            if (graph.nodes.count(label1)) {
                graph.DFS(label1);
            } else {
                cout << "The node is not found";
            }
            cout << endl;
        } else if (command == 7) {
            bool t = graph.isCyclic();
            if (t) {
                cout << "Graph is cyclic.";
            } else {
                cout << "Graph is not cyclic.";
            }
            cout << endl;
        } else if (command == 8) {
            bool t;
            for (auto it : graph.nodes) {
                t = graph.isCyclic();
                if (graph.isCyclic() || !isDirected) {
                    cout << "Can't perform Topological Sort on undirected or cyclic graph" << endl;
                }
                break;
            }
            if (!t) {
                graph.topologicalSort();
            }
            cout << endl;
        } else if (command == 9) {
            cout << "Enter the labels of the two nodes: ";
            string label1, label2;
            cin >> label1 >> label2;
            if (graph.nodes.count(label1) && graph.nodes.count(label2)) {
                if (negWeight) {
                    graph.bellmanFord(label1, label2);
                } else {
                    graph.Dijkstra(label1, label2);
                }
            } else {
                cout << "One or both nodes are not found";
            }
            cout << endl;
        }
    }
    for (auto nodePair : graph.nodes) {
        Node* node = nodePair.second;
        cout << "Node " << node->label << " has neighbors: ";
        for (auto neighborPair : node->neighbors) {
            Node* neighbor = neighborPair.first;
            float weight = neighborPair.second;
            cout << neighbor->label << " (" << weight << ") ";
        }
        cout << endl;
    }
    return 0;
}
