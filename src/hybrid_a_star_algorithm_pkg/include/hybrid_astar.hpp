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
std::vector<double> adj_nodes_x2;
std::vector<double> adj_nodes_y2;
std::vector<double> adj_nodes_x3;
std::vector<double> adj_nodes_y3;

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
        double xn;  // n -> now
        double yn;
        double xc;  // c -> center
        double yc;
    };
    struct Cost cost;
    struct Pos pos;
    int cur_address;
    int parent_address;
};

// Process : NewNode -> 반올림(node[i].xn, node[i].yn)의 closed_flag -> 진행
// x = 반올림(node[i].xn)
// y = 반올림(node[i].yn)
// if(op[x][y].closed == true) ?
//  stop to propagate
// else
//  go propagation

// NewNode 증식 후에,
// x = 반올림(node[i].xc)
// y = 반올림(node[i].yc)
// op[x][y].closed = true;

// op[x][y].closed 는 어떻게 구성?
// x, y의 크기는 충분히 커야함.
// occupancy_points_flag[x][u] = 0;
// Node 안에 adj_occupancy_point 로 이동.
// 해당 point가 closed 되어있는지 확인.
// 기존 closed는? 제거.>?
class HybridAstar
{
public:
    std::vector<struct Node> node; // Node

    // Hybrid A*의 Node Renew는 Continous함. 따라서 Closed_flag를 위하여 Discrete한 Occupancy Grid Points를 사용.
    // oc_closed_flag가 1이면, 인접 Node들이 Closed Node 상태를 의미.
    // Renewed Node는 소수점 반올림을 통해 oc_closed_flag가 정의됨.
    std::vector<std::vector<bool>> oc_closed_flag;
    int oc_size_row;
    int oc_size_col;
    int negative_val_offset;

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
        : threshold_distance_for_stop(15),
          init_pos_index(0),
          oc_size_row(50000),
          oc_size_col(60000),
          negative_val_offset(4000)
    {
        // ColxRow 크기의 Occuapcny Grid Map 선언 및 Closed Node 상태를 False로 선언.
        // oc_closed_flag[x][y] 이고, x의 사이즈는 oc_size_row, y의 사이즈는 oc_size_col
        oc_closed_flag.assign(oc_size_row, std::vector<bool>(oc_size_col, false));

        target_x = target_x_;
        target_y = target_y_;

        Node init_node_tmp;
        init_node_tmp.pos = {50, 50, 20, 50};
        // init_node_tmp.ptr_parent_node = NULL;
        init_node_tmp.cur_address = 0;
        init_node_tmp.parent_address = 0;
        // init_node_tmp.close_flag = false;
        init_node_tmp.cost.g = 0;
        init_node_tmp.cost.h = std::sqrt(pow(init_node_tmp.pos.xn - this->target_x, 2) + std::pow(init_node_tmp.pos.yn - this->target_y, 2));
        init_node_tmp.cost.f = init_node_tmp.cost.h + init_node_tmp.cost.g;

        // 초기 노드를 선언함
        this->node.push_back(init_node_tmp);

        // FIXME: delete
        test = cv::imread("../optimization_class_ws/img/1.jpeg");
        test = cv::Scalar(255, 255, 255);
        cv::resize(test, test, cv::Size(1000, 1000));

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
            // Call the Closed State of Nodes
            int oc_x_tmp = std::round(this->node[i].pos.xn);
            int oc_y_tmp = std::round(this->node[i].pos.yn);
            if (oc_x_tmp < 0)
                oc_x_tmp += this->negative_val_offset;
            if (oc_y_tmp < 0)
                oc_y_tmp += this->negative_val_offset;

            // Open Node List 조건
            // if (this->node[i].close_flag == false)
            if (this->oc_closed_flag[oc_x_tmp][oc_y_tmp] == false)
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
        // if (this->node[cur_i].cost.h < this->threshold_distance_for_stop) // cost.h는 목표지점까지의 uc_distance
        double tmp_thresh = EuclideanDistance(this->node[cur_i].pos.xn - target_x, this->node[cur_i].pos.yn - target_y);
        if (tmp_thresh < this->threshold_distance_for_stop) // cost.h는 목표지점까지의 uc_distance
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
        node.insert(node.end(), updated_node_tmp.begin(), updated_node_tmp.end());  // 주의 : vector의 insert, push_back 사용 시시 기존 주소가 변함함.

        for (int i = 0; i < node.size(); i++)
        {
            test.data[((int)node[i].pos.xn * 3 + 0) + (3 * (int)node[i].pos.yn * test.cols)] = 0;
            test.data[((int)node[i].pos.xn * 3 + 1) + (3 * (int)node[i].pos.yn * test.cols)] = 0;
            test.data[((int)node[i].pos.xn * 3 + 2) + (3 * (int)node[i].pos.yn * test.cols)] = 255;
        }

        adj_nodes_x2.push_back(this->node[cur_i].pos.xn);
        adj_nodes_y2.push_back(this->node[cur_i].pos.yn);
        for (int i = 0; i < adj_nodes_x2.size(); i++)
        {
            cv::circle(test, cv::Point(adj_nodes_x2[i], adj_nodes_y2[i]), 2, cv::Scalar(100, 100, 0), 1);
        }
        
        cv::imshow("test", test);
        cv::imwrite("../optimization_class_ws/src/hybrid_a_star_algorithm_pkg/result.jpg", test);
        cv::waitKey(1);
        test.setTo(255);
        usleep(10);
        // usleep(500000);
        

        // (A) 위치에서 생성한 Neighborhood Nodes들 중, 장애물을 맞닥드린 Nodes는 제거.
        // TODO: (A) 위치에서 생성한 Neighborhood Nodes들 중, 장애물을 맞닥드린 Nodes는 제거.
        // FIXME:
        // for (int i = 0; i < this->node.size(); i++)
        // {
        //     // Call the Closed State of Nodes
        //     int oc_x_tmp = std::round(this->node[i].pos.xn);
        //     int oc_y_tmp = std::round(this->node[i].pos.yn);
        //     if (oc_x_tmp < 0) oc_x_tmp += this->negative_val_offset;
        //     if (oc_y_tmp < 0) oc_y_tmp += this->negative_val_offset;
            
        //     // Red
        //     // adj_nodes_x.push_back(node[i].pos.xc);
        //     // adj_nodes_y.push_back(node[i].pos.yc);

        //     if (this->oc_closed_flag[oc_x_tmp][oc_y_tmp] == true)
        //     {
        //         // Green
        //         // adj_nodes_x2.push_back(node[i].pos.xc);
        //         // adj_nodes_y2.push_back(node[i].pos.yc);
        //         // FIXME: Set to visualize
        //         // FIXME: Set to visualize
        //         adj_nodes_x.push_back(node[i].pos.xn);
        //         adj_nodes_y.push_back(node[i].pos.yn);

        //         // Gray
        //         // adj_nodes_x3.push_back(oc_x_tmp);
        //         // adj_nodes_y3.push_back(oc_y_tmp);
        //     }
        // }

        // TODO: Hybrid A*는 겹치는 부분에 의해 Cost가 다시 갱신되는 경우가 없나?? 어떻게 하나?? 그냥 무시하고??
        // -> 무시해도 될 듯. 이 부분이 연산량을 늘리는 부분일 듯 함. 중복되는 경우가 있다면, Kinematic Model 계산에 의해 0.00x 이상 차이가 날 것.

        // Call the Closed State of Nodes
        int oc_x_tmp = std::round(this->node[cur_i].pos.xn);
        int oc_y_tmp = std::round(this->node[cur_i].pos.yn);
        if (oc_x_tmp < 0) 
        {
            std::cout << "Negative value for oc_x_tmp" << std::endl;   
            oc_x_tmp += this->negative_val_offset;
        }
        if (oc_y_tmp < 0) 
        {
            std::cout << "Negative value for oc_y_tmp" << std::endl;   
            oc_y_tmp += this->negative_val_offset;
        }

        // (A)를 OpenNodeList에서 제거
        this->oc_closed_flag[oc_x_tmp][oc_y_tmp] = true;

        // FIXME:
        // adj_nodes_x3.push_back(this->node[cur_i].pos.xn);    // Gray
        // adj_nodes_y3.push_back(this->node[cur_i].pos.yn);
        // adj_nodes_x2.push_back(oc_x_tmp);    // Green
        // adj_nodes_y2.push_back(oc_y_tmp);
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
        
        // Default
        // radius.push_back(-3.20);
        // radius.push_back(-8.00);
        // radius.push_back(100000);
        // radius.push_back(8.00);
        // radius.push_back(3.20);
        

        // New (Test)
        // FIXME: 생성할 Node의 개수를 골라야함.
        // radius.push_back(-3.20);
        // radius.push_back(-8.00);
        // radius.push_back(-12.00);
        // radius.push_back(-25.00);
        // radius.push_back(-50.00);
        // radius.push_back(100000);
        // radius.push_back(50.00);
        // radius.push_back(25.00);
        // radius.push_back(12.00);
        // radius.push_back(8.00);
        // radius.push_back(3.20);

            // New (Test)
        // FIXME: 생성할 Node의 개수를 골라야함.
        radius.push_back(-3.20);
        radius.push_back(-8.00);
        radius.push_back(-12.00);
        radius.push_back(100000);
        radius.push_back(12.00);
        radius.push_back(8.00);
        radius.push_back(3.20);
        double length_arc = 8; //0.97; // 1.6
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
        // for (int i = 0; i < updated_node_list_tmp.size(); i++)
        // {
        //     updated_node_list_tmp[i].cost.g = stand_node->cost.g + length_arc / cost_g_offset; // FIXME: /cost_g_offset 개념 알아두기.
        //     updated_node_list_tmp[i].cost.h = std::sqrt(pow(updated_node_list_tmp[i].pos.xn - this->target_x, 2) + std::pow(updated_node_list_tmp[i].pos.yn - this->target_y, 2));
        //     updated_node_list_tmp[i].cost.f = updated_node_list_tmp[i].cost.g + updated_node_list_tmp[i].cost.h;
        //     updated_node_list_tmp[i].cur_address = this->node.size() + i;
        //     updated_node_list_tmp[i].parent_address = stand_node->cur_address;
        //     // updated_node_list_tmp[i].close_flag = false;
        // }

        // With Potential Field        
        for (int i = 0; i < updated_node_list_tmp.size(); i++)
        {
            double U_att = 0; // attractive potential
            double U_rep = 0; // repulsive potential
            double U_total = 0;
            double ego_x_tmp = updated_node_list_tmp[i].pos.xn;
            double ego_y_tmp = updated_node_list_tmp[i].pos.yn;

            // 경사 생성
            // U_att += -PotentialExp(ego_x_tmp, ego_y_tmp)/10.;
            // WPs 넣기 (0,0), (1,1) ..., (99,99) 의 WPs (총 100개)가 있다고 가정하고 테스트.
            // for(int i = 10; i < 200; i++)
            // {
            //     // U_att += PotentialExp(ego_x_tmp-i, ego_y_tmp-i)*i;
            //     U_att += PotentialDistance(ego_x_tmp-i, ego_y_tmp-i)*i/100000.;
            //     // std::cout << U_att << "\t\t" << ego_x_tmp << "\t\t" << ego_y_tmp << std::endl;
            // }
            
            U_att += EuclideanDistance(ego_x_tmp-25, ego_y_tmp-100)*EuclideanDistance(25, 100)/1000.;
            U_att += EuclideanDistance(ego_x_tmp-50, ego_y_tmp-200)*EuclideanDistance(50, 200)/1000.;
            
            // U_total = U_rep - U_att;

            // updated_node_list_tmp[i].cost.g = stand_node->cost.g + length_arc / cost_g_offset; // FIXME: /cost_g_offset 개념 알아두기.
            // updated_node_list_tmp[i].cost.h = std::sqrt(pow(updated_node_list_tmp[i].pos.xn - this->target_x, 2) + std::pow(updated_node_list_tmp[i].pos.yn - this->target_y, 2));
            updated_node_list_tmp[i].cost.h = U_att;// + stand_node->cost.h;
            updated_node_list_tmp[i].cost.f = updated_node_list_tmp[i].cost.g + updated_node_list_tmp[i].cost.h;
            std::cout << updated_node_list_tmp[i].cost.g << "\t\t" << updated_node_list_tmp[i].cost.h << "\t\t[" << updated_node_list_tmp[i].cost.f << "]\t\t" << U_rep << "\t\t[" << U_att << std::endl;
            updated_node_list_tmp[i].cur_address = this->node.size() + i;
            updated_node_list_tmp[i].parent_address = stand_node->cur_address;
            // updated_node_list_tmp[i].close_flag = false;
        }


        return updated_node_list_tmp;
    }
    double EuclideanDistance(double x_, double y_)
    {
        return std::sqrt(x_*x_ + y_*y_);
    }
    double PotentialExp(double x_, double y_)
    {
        return std::exp(-std::sqrt(x_*x_ + y_*y_));
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

        if (this->node[init_pos_index].cur_address != arrival_node_->cur_address)
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
