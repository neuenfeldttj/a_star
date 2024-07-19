#include <cstdint>
#include <vector>
#include <cmath>
#include <queue>
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
                double pred_cost = heuristic(n->x, n->y, goal_x, goal_y, &euclid_square); //faster because no sqrt
                if(expected_cost + pred_cost < n->cost) {
                    n->cost = expected_cost + pred_cost;
                    n->parent = ptr;
                    pq.push(n);
                }
            }
        }
    }

    // Backtrack through path, adding to vector
    vector<cv::Point> path;
    Node* ptr = nodes[goal_y*num_cols+goal_x];
    while(ptr){
        path.emplace_back(ptr->x, ptr->y);
        ptr = ptr->parent;
    }
    reverse(path.begin(), path.end()); //so it goes from start -> end
    return path;
}

/* Commented out lines were used to determine HSV range*/
// int H_L = 0, S_L = 0, V_L = 0;
// int H_H = 0, S_H = 0, V_H = 0;

// cv::Mat image, image_hsv;

// void show_mask() {
//     cv::inRange(image, cv::Scalar(H_L, S_L, V_L), cv::Scalar(H_H, S_H, V_H), image_hsv);
//     cv::imshow("Mask", image_hsv);
// }

// void h_l(int hue, void*) {
//     H_L = hue;
//     show_mask();
// }
// void h_h(int hue, void*) {
//     H_H = hue;
//     show_mask();
// }
// void s_l(int hue, void*) {
//     S_L = hue;
//     show_mask();
// }
// void s_h(int hue, void*) {
//     S_H = hue;
//     show_mask();
// }
// void v_l(int hue, void*) {
//     V_L = hue;
//     show_mask();
// }
// void v_h(int hue, void*) {
//     V_H = hue;
//     show_mask();
// }


void onMouse(int event, int x, int y, int, void*) {
    if( event != cv::EVENT_LBUTTONDOWN ) return;
    cout << "X: " << x << ", Y: " << y << endl;
}


int main(int argc, char** argv) {

    /*
        Command line arguments:
        image filename
        start_x
        stary_y
        goal_x
        goal_y
    */

    cv::Mat image_orig;
    image_orig = cv::imread(argv[1], cv::IMREAD_COLOR);
    if(image_orig.empty()) {
        cout << "Cannot read in image!" << endl;
        exit(1);
    }

    int start_x = atoi(argv[2]), start_y = atoi(argv[3]);
    int goal_x = atoi(argv[4]), goal_y = atoi(argv[5]);

    if(start_x < 0 || start_x >= image_orig.cols) {
        cout << "start_x invalid" << endl;
        exit(1);
    }
    if(start_y < 0 || start_y >= image_orig.rows) {
        cout << "start_y invalid" << endl;
        exit(1);
    }
    if(goal_x < 0 || goal_x >= image_orig.cols) {
        cout << "goal_x invalid" << endl;
        exit(1);
    }
    if(goal_y < 0 || goal_y >= image_orig.rows) {
        cout << "goal_y invalid" << endl;
        exit(1);
    }

    cv::Mat image;
    cv::cvtColor(image_orig, image, cv::COLOR_BGR2HSV);

    // cv::imshow("HSV", image);
    // cv::imshow("Mask", image);

    // cv::createTrackbar("H_L", "Mask", &H_L, 180, h_l);
    // cv::createTrackbar("H_H", "Mask", &H_H, 180, h_h);
    // cv::createTrackbar("S_L", "Mask", &S_L, 255, s_l);
    // cv::createTrackbar("S_H", "Mask", &S_H, 255, s_h);
    // cv::createTrackbar("V_L", "Mask", &V_L, 255, v_l);
    // cv::createTrackbar("V_H", "Mask", &V_H, 255, v_h);
    
    cv::Mat image_boxes, image_forklifts;
    cv::inRange(image, cv::Scalar(8, 46, 116), cv::Scalar(28, 255, 255), image_boxes);
    cv::inRange(image, cv::Scalar(0, 0, 0), cv::Scalar(180, 0, 95), image_forklifts);
    image = image_boxes | image_forklifts;
    cv::erode(image, image, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1,-1), 2);
    // cv::imshow("Erode", image);
    cv::dilate(image, image, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)), cv::Point(-1,-1), 5);
    // cv::imshow("Dilate", image);
    // cv::setMouseCallback("Dilate", onMouse, 0);
    // cv::waitKey(0);
    // return 0;

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

    vector<cv::Point> path = a_star(start_x, start_y, goal_x, goal_y, nodes, image.cols);

    for(size_t i = 1; i < path.size(); ++i) {
        cv::line(image_orig, path[i-1], path[i], cv::Scalar(255,0,0), 3, cv::LINE_4);
    }

    cv::imwrite("output_" + to_string(start_x) + "_" + to_string(start_y) + "_" + to_string(goal_x) + "_" + to_string(goal_y) + ".png", image_orig);

    for(auto n : nodes) {
        delete n;
    }

}