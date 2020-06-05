#ifndef hybrid_astar_hpp
#define hybrid_astar_hpp


#include <iostream>
#include <vector>
#include <cmath>
#include <unistd.h>


// FIXME: Set to Visualize
// FIXME: Set to Visualize
// FIXME: Set to Visualize
// FIXME: Set to Visualize
// FIXME: Set to Visualize
// For visualizer
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
cv::Mat test;
std::vector<double> adj_nodes_x;
std::vector<double> adj_nodes_y;

struct Node
{
    struct Cost
    {
        double g; // The sum of the calcuated costs
        double h; // The cost which will be spend
        double f; // The sum of the g and h
    };
    struct Pos
    {
        double xn;
        double yn;
        double xc;
        double yc;
    };
    struct Cost cost;
    struct Pos pos;
    bool close_flag;
    int cur_address;
    int parent_address;
};

class HybridAstar
{
public:
    std::vector<struct Node> node;                 // Node


    // Result
    std::vector<double> local_x;
    std::vector<double> local_y;
    int grid_width;
    int grid_height;
    int cur_pos_x;
    int cur_pos_y;
    int init_pos_index;
    double threshold_distance_for_stop;

    double target_x;
    double target_y;

    HybridAstar(double target_x_, double target_y_) 
    : threshold_distance_for_stop(15), init_pos_index(0)
    {
        target_x = target_x_;
        target_y = target_y_;

        Node init_node_tmp;
        init_node_tmp.pos = {50, 50, 20, 50};
        // init_node_tmp.ptr_parent_node = NULL;
        init_node_tmp.cur_address = 0;
        init_node_tmp.parent_address = 0;
        init_node_tmp.close_flag = false;
        init_node_tmp.cost.g = 0;
        init_node_tmp.cost.h = std::sqrt(pow(init_node_tmp.pos.xn - this->target_x, 2) + std::pow(init_node_tmp.pos.yn - this->target_y, 2));
        init_node_tmp.cost.f = init_node_tmp.cost.h + init_node_tmp.cost.g;
        
        // 초기 노드를 선언함
        this->node.push_back(init_node_tmp);



        // FIXME: delete        
        test = cv::imread("../optimization_class_ws/img/1.jpeg");
        test = cv::Scalar(255, 255, 255);
        cv::resize(test, test, cv::Size(100, 100));
        
        std::cout << "initialized" << std::endl;
    }

    ~HybridAstar()
    {
    }



    bool Run()
    {

        // Open Node List 중 가장 작은 cost.f값의 인덱스 구하기 (A)
        double tmp = 65000.;
        int cur_i = 0;
        for (int i = 0; i < this->node.size(); i++)
        {
            // Open Node List 조건
            if (this->node[i].close_flag == false)
            {
                if (tmp > this->node[i].cost.f)
                {
                    tmp = this->node[i].cost.f;
                    cur_i = i;
                }
            }
        }

        // 최종 결과 반환
        // If (Open Node List). node의 Index cur_i는 open_node_list일 수 밖에 없음.
        if (this->node[cur_i].cost.h < this->threshold_distance_for_stop) // cost.h는 목표지점까지의 uc_distance
        {
            // Cost값 0을 갖고있는 초기 위치가 아닐 때,
            if (&this->node[cur_i] != &this->node[init_pos_index])
            {
                // Local Path 반환
                // 초기 위치의 주소 : &this->node[init_pos_index].cur_address
                // 현재 위치의 주소 : this->node[cur_i].cur_address
                // 현재-1 위치의 주소 : this->node[cur_i]->parent_address
                ReturnPath(&this->node[cur_i]);

                return true;
            }
        }


        // (A)위치에서 Kinematic Model 기반의 Neighborhood Nodes를 구하고, node에에 넣음
        std::vector<struct Node> updated_node_tmp = UpdateNode(&this->node[cur_i]); // Set New Node
        node.insert(node.end(), updated_node_tmp.begin(), updated_node_tmp.end()); // 주의 : vector의 insert, push_back 사용 시시 기존 주소가 변함함.


        for(int i = 0; i < node.size(); i++)
        {
            test.data[((int)node[i].pos.xn*3+0) + (3*(int)node[i].pos.yn*test.cols)] = 0;
            test.data[((int)node[i].pos.xn*3+1) + (3*(int)node[i].pos.yn*test.cols)] = 0;
            test.data[((int)node[i].pos.xn*3+2) + (3*(int)node[i].pos.yn*test.cols)] = 255;
        }


        cv::imshow("test", test);
        cv::imwrite("../optimization_class_ws/src/hybrid_a_star_algorithm_pkg/result.jpg", test);
        cv::waitKey(10);
        test.setTo(255);
        usleep(50000);

        // (A) 위치에서 생성한 Neighborhood Nodes들 중, 장애물을 맞닥드린 Nodes는 제거.
        // TODO: (A) 위치에서 생성한 Neighborhood Nodes들 중, 장애물을 맞닥드린 Nodes는 제거.
        for(int i = 0; i < this->node.size(); i++)
        {
            if (this->node[i].close_flag == true)
            {
                // FIXME: Set to visualize
                // FIXME: Set to visualize
                adj_nodes_x.push_back(node[i].pos.xn);
                adj_nodes_y.push_back(node[i].pos.yn);
            }
        }

        // TODO: Hybrid A*는 겹치는 부분에 의해 Cost가 다시 갱신되는 경우가 없나?? 어떻게 하나?? 그냥 무시하고??
        // -> 무시해도 될 듯. 이 부분이 연산량을 늘리는 부분일 듯 함. 중복되는 경우가 있다면, Kinematic Model 계산에 의해 0.00x 이상 차이가 날 것.

        // (A)를 OpenNodeList에서 제거
        // ttttt
        this->node[cur_i].close_flag = true;


        return false;
    }

    std::vector<Node> UpdateNode(Node *stand_node)
    {
        /////////////////////
        ///// Parameter /////
        /////////////////////
        std::vector<double> radius;
        // -30, -20, -10, 0, 10, 20, 30 Deg
        // double radius[] = {-4.16, -6.59, -13.61, 10000, 13.61, 6.59, 4.16};
        // 일반 대형세단 회전반경이 5.0~6.0m 정도 됨.
        // radius.push_back(-4.16); radius.push_back(-6.59); radius.push_back(-13.61); radius.push_back(1000); 
        // radius.push_back(13.61); radius.push_back(6.59); radius.push_back(4.16);
        // radius.push_back(-4.16); radius.push_back(-8.96); radius.push_back(1000); 
        // radius.push_back(8.96); radius.push_back(4.16);
        radius.push_back(-3.20); radius.push_back(-8.00); radius.push_back(1000); 
        radius.push_back(8.00); radius.push_back(3.20);
        double length_arc = 0.97;
        double cost_g_offset = 2;

        
        //////////////////////////////
        //// Set Node index & Pos ////
        //////////////////////////////
        // Input  : {init_xn, init_yn}, {init_xc, init_yc}, dir(얘는 후진의 경우?)
        // Output : {Xn_new, Yn_new, Xc_new, Yc_new}
        std::vector<Node> updated_node_list_tmp;
        Node init_tmp;
        init_tmp = *stand_node;
        for (int i = 0; i < radius.size(); i++)
        {
            for (int j = 0; j < 2; j++)
            {
                Node tmp = {};
                tmp.pos = ReturnPOSnc({init_tmp.pos.xn, init_tmp.pos.yn, init_tmp.pos.xc, init_tmp.pos.yc}, radius[i], length_arc); // At this, pos_c mean heading.
                updated_node_list_tmp.push_back(tmp);

                // Rewnew POSc (Including Heading info)
                init_tmp.pos.xc = tmp.pos.xc;
                init_tmp.pos.yc = tmp.pos.yc;
            }

        }

        ////////////////////////////////
        /// Set Node Cost and Parent ///
        ////////////////////////////////
        for(int i = 0; i < updated_node_list_tmp.size(); i++)
        {
            updated_node_list_tmp[i].cost.g = stand_node->cost.g + length_arc/cost_g_offset; // FIXME: /cost_g_offset 개념 알아두기.
            updated_node_list_tmp[i].cost.h = std::sqrt(pow(updated_node_list_tmp[i].pos.xn - this->target_x, 2) + std::pow(updated_node_list_tmp[i].pos.yn - this->target_y, 2));
            updated_node_list_tmp[i].cost.f = updated_node_list_tmp[i].cost.g + updated_node_list_tmp[i].cost.h;
            updated_node_list_tmp[i].cur_address = this->node.size() + i;
            updated_node_list_tmp[i].parent_address = stand_node->cur_address;
            updated_node_list_tmp[i].close_flag = false;
        }


        return updated_node_list_tmp;
    }

    Node::Pos ReturnPOSnc(Node::Pos pos_, double radius_, double length_arc_)
    {
        Node::Pos pos_new_tmp;

        // POSn, Heading(POSr), radius, length로 POSn+1, Heading(POSr+1) 구하기
        double k = -1;
        // if (dir_change_flag_ == true)
        //     k = 1 + std::sqrt(radius_ * radius_ / ((POSn_.x - POSc_.x) * (POSn_.x - POSc_.x) + (POSn_.y - POSc_.y) * (POSn_.y - POSc_.y)));
        // else
        //     k = 1 - std::sqrt(radius_ * radius_ / ((POSn_.x - POSc_.x) * (POSn_.x - POSc_.x) + (POSn_.y - POSc_.y) * (POSn_.y - POSc_.y)));
        k = 1 + std::sqrt(radius_ * radius_ / ((pos_.xn - pos_.xc) * (pos_.xn - pos_.xc) + (pos_.yn - pos_.yc) * (pos_.yn - pos_.yc)));

        pos_new_tmp.xc = (pos_.xn - pos_.xc) * k + pos_.xc;
        pos_new_tmp.yc = (pos_.yn - pos_.yc) * k + pos_.yc;

        double theta_k = length_arc_ / radius_ * -1;
        pos_new_tmp.xn = std::cos(theta_k) * (pos_.xn - pos_new_tmp.xc) - std::sin(theta_k) * (pos_.yn - pos_new_tmp.yc) + pos_new_tmp.xc;
        pos_new_tmp.yn = std::sin(theta_k) * (pos_.xn - pos_new_tmp.xc) + std::cos(theta_k) * (pos_.yn - pos_new_tmp.yc) + pos_new_tmp.yc;

        return pos_new_tmp;
    }

    void ReturnPath(struct Node *arrival_node_)
    {
        std::cout << "return pos : [" << arrival_node_->pos.xn << ", " << arrival_node_->pos.yn << "] \t\t node[init]_address : " << this->node[init_pos_index].cur_address << "\t\t ptr : " << arrival_node_->cur_address << std::endl;

        if(this->node[init_pos_index].cur_address != arrival_node_->cur_address)
        {
            local_x.push_back(arrival_node_->pos.xn);
            local_y.push_back(arrival_node_->pos.yn);

            ReturnPath(&this->node[arrival_node_->parent_address]);
        }
    }


protected:
private:
};


#endif /* hybrid_astar_hpp */
