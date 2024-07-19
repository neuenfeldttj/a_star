#include <cstdint>
#include <vector>
#include <cmath>
#include <queue>
#include <string>
#include <functional>

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;

struct Node {
    uint8_t val;
    int x;
    int y;
    double cost;
    bool visited;
    Node* parent;
    vector<Node*> neighbors;
};

struct PQComp {
    bool operator()(const Node* a, const Node* b) const {
        return a->cost > b->cost;
    }
};


double euclid(int s_x, int s_y, int g_x, int g_y) { return sqrt((g_x-s_x)*(g_x-s_x) + (g_y-s_y)*(g_y-s_y)); }

double euclid_square(int s_x, int s_y, int g_x, int g_y) { return (g_x-s_x)*(g_x-s_x) + (g_y-s_y)*(g_y-s_y); }

double l1_norm(int s_x, int s_y, int g_x, int g_y) { return abs(g_x-s_x) + abs(g_y-s_y); }

double heuristic(int start_x, int start_y, int goal_x, int goal_y, function<double(int, int, int, int)> func){
    return func(start_x, start_y, goal_x, goal_y);
}


vector<cv::Point> a_star(int start_x, int start_y, int goal_x, int goal_y, vector<Node*> &nodes, int num_cols){
    //A* algorithm
    priority_queue<Node*, vector<Node*>, PQComp> pq;
    nodes[start_y*num_cols+start_x]->cost = 0;
    pq.push(nodes[start_y*num_cols+start_x]);

    while(!pq.empty()){
        Node* ptr = pq.top();
        pq.pop();
        if(!ptr->visited) {
            ptr->visited = true;
            for(auto n : ptr->neighbors){
                double expected_cost = ptr->cost + 1; //distance is 1 pixel over so cost is +1
                double pred_cost = heuristic(n->x, n->y, goal_x, goal_y, &euclid_square);
                if(expected_cost + pred_cost < n->cost) {
                    n->cost = expected_cost + pred_cost;
                    n->parent = ptr;
                    pq.push(n);
                }
            }
        }
    }

    vector<cv::Point> path;
    Node* ptr = nodes[goal_y*num_cols+goal_x];
    while(ptr){
        path.emplace_back(ptr->x, ptr->y);
        ptr = ptr->parent;
    }
    reverse(path.begin(), path.end());
    return path;
}



int main() {
    
    const string image_path = "test_2.png";
    cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    vector<Node*> nodes(image.rows*image.cols);

    cout << "Read in image" << endl;
    for(int i = 0; i < image.rows; ++i) {
        const uint8_t* row = image.ptr<uint8_t>(i);
        for(int j = 0; j < image.cols; ++j) {
            Node* ptr = new Node();
            ptr->val = row[j];
            ptr->x = j;
            ptr->y = i;
            ptr->cost = numeric_limits<double>::infinity();
            ptr->visited = false;
            ptr->parent = nullptr;
            nodes[i*image.cols+j] = ptr;
        }
    }

    for(int i = 0; i < image.rows; ++i) {
        for(int j = 0; j < image.cols; ++j) {
            Node* ptr = nodes[i*image.cols+j];
            if(j > 0 && !nodes[i*image.cols+j-1]->val) ptr->neighbors.push_back(nodes[i*image.cols+j-1]);
            if(j < image.cols-1 && !nodes[i*image.cols+j+1]->val) ptr->neighbors.push_back(nodes[i*image.cols+j+1]);
            if(i > 0 && !nodes[(i-1)*image.cols+j]->val) ptr->neighbors.push_back(nodes[(i-1)*image.cols+j]);
            if(i < image.rows-1 && !nodes[(i+1)*image.cols+j]->val) ptr->neighbors.push_back(nodes[(i+1)*image.cols+j]);
        }
    }

    cout << "Set up graph. Computing A*" << endl;

    vector<cv::Point> path = a_star(0, 0, 7, 7, nodes, image.cols);

    for(size_t i = 1; i < path.size(); ++i) {
        cv::line(image, path[i-1], path[i], cv::Scalar(255,0,0), 1, cv::LINE_4);
    }

    cv::imwrite("output.png", image);

    for(auto n : nodes) {
        delete n;
    }

}