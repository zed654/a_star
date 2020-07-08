#include <iostream>
#include <string>

// For Hybrid Astar
#include "hybrid_astar.hpp"

// For visualizer
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char **argv)
{

    // FIXME: have to delete
    //    double wb = 2.75;                       // unit : m     휠베이스
    //    double steer_angle_tmp = 450;           // unit : deg   스티어링 각도
    //    double d_mv = 5;                        // unit : m     예측할 이동거리

    //    // d_mv <= 2 Unit Grid. Unit Grid -> 0.1m

    //    // 스티어링 변화에 따라, 회전하는 방향으로 검출ROI의 영역을 더 늘리기 위한 offset임.
    //    double steer_rot_offset = steer_angle_tmp / 25.;

    //    // 차량 외곽 크기를 기준으로 y축에 offset 0.4씩 넣어준 것 (그래서 0.7, -0.7이 된 것.)
    //    double ref_x[4] = {2.1, 2.1, -2.1, -2.1};
    //    double ref_y[4] = {0.7, -0.7, -1.1, 1.1};
    //    double l = 0;     // bycycle model 에서 회전중심과 뒷바퀴와의 거리
    //    double theta = 0; // bycycle modle의 회전중심을 기준으로 d_mv만큼 호를 그리며 이동할 때, 이동한 거리에 따른 각도

    //    double pred_x[4] = {}; // 결과
    //    double pred_y[4] = {}; // 결과

    //    // l, theta 계산
    //    l = wb / std::tan(steer_angle_tmp / 180. * M_PI);
    //    theta = d_mv / l * M_PI / 180.;

    //    // Rotation and Translation
    //    for (int i = 0; i < 4; i++)
    //    {
    //        pred_x[i] = std::cos(theta * 180 / M_PI) * ref_x[i] - std::sin(theta * 180 / M_PI) * (ref_y[i] - l);
    //        pred_y[i] = std::sin(theta * 180 / M_PI) * ref_x[i] + std::cos(theta * 180 / M_PI) * (ref_y[i] - l) + l;
    //    }

    // Input : Steer Angle, Yaw Angle
    // Output : Predicted PosXY
    double yaw_angle = 10; // Const
    double wb = 4.8;
    double steer_angle_rag = 5 * M_PI / 180.;
    double radius = wb * std::sqrt(1 / std::pow(steer_angle_rag, 2) - 1. / 4.);

    ///////////////////////////////
    ///////////////////////////////
    ///////////////////////////////
    ///////////////////////////////
    if (argc != 6)
    {
        std::cout << "The command should be comply with this format." << std::endl;
        std::cout << "Format : [img_num] [init_x] [init_y] [target_x] [target_y]" << std::endl;
        std::cout << "Ex : rosrun a_star_algorithm_pkg a_star_algorithm_pkg_exe [img_num] [init_x] [init_y] [final_x] [final_y]" << std::endl;
        exit(1);
    }

    ////////////////////////
    //////// Praram ////////
    ////////////////////////
    // Set Input Image Name (1~7)
    std::string img_count = argv[1];

    // Set Size of Obstacle Image
    int obstacle_img_width = 960;
    int obstacle_img_height = 540;

    // Set Size of Occupancy Grid Map, Astar
    int occupancy_grid_map_width = 959;
    int occupancy_grid_map_height = 538;

    // Set Departure Pos_xy
    // int init_pos_x = std::stoi(argv[2]) + 1;
    // int init_pos_y = obstacle_img_height - 1 - std::stoi(argv[3]);
    int init_pos_x = std::stoi(argv[2]);
    int init_pos_y = std::stoi(argv[3]);

    // Set Arrival Pos_xy
    // int target_x = std::stoi(argv[4]) + 1;
    // int target_y = obstacle_img_height - 1 - std::stoi(argv[5]);
    int target_x = std::stoi(argv[4]);
    int target_y = std::stoi(argv[5]);

    // Clipping for input pos_xy
    if (init_pos_x<3 | init_pos_y<3 | target_x> 958 | target_y> 540)
    {
        std::cout << "The pos should be larget than (3, 3) and smaller than (958, 540)" << std::endl;
        exit(1);
    }

    int k = 0;
    while (cv::waitKey(10) != 'q')
    {
        k++;
        adj_nodes_x.clear();
        adj_nodes_y.clear();
        adj_nodes_x2.clear();
        adj_nodes_y2.clear();
        adj_nodes_x3.clear();
        adj_nodes_y3.clear();
        ////////////////////////////////////
        ////////////////////////////////////
        ////////////////////////////////////
        //////// Set Obstacle Image ////////
        ////////////////////////////////////
        ////////////////////////////////////
        ////////////////////////////////////
        // Get Input Img from jpeg
        cv::Mat obstacle_img;
        obstacle_img = cv::imread("../optimization_class_ws/img/" + img_count + ".jpeg");
        if (!obstacle_img.data)
        {
            std::cout << "Can't find the image file" << std::endl;
            exit(1);
        }
        cv::resize(obstacle_img, obstacle_img, cv::Size(obstacle_img_width, obstacle_img_height));

        // OpenCV Coordinate -> Occupancy Grid Map Coordinate
        // OpenCV와 Astar의 Occuapncy Grid Map의 좌표계가 다르기 때문에 상하반전 작업
        // cv::flip(obstacle_img, obstacle_img, 0);

        // Set Result Image to visualize
        cv::Mat result;
        result = obstacle_img.clone();

        //////////////////////////////
        //////////////////////////////
        //////////////////////////////
        //////// Hybrid Astar ////////
        //////////////////////////////
        //////////////////////////////
        //////////////////////////////
        // Set Astar
        // HybridAstar h_astar(occupancy_grid_map_width, occupancy_grid_map_height, init_pos_x, init_pos_y);
        // HybridAstar h_astar(target_x + std::sin(k) * 10, target_y + std::sin(k) * 10);
        HybridAstar h_astar(target_x + std::sin(k) * 5, target_y);
        // h_astar.TEST();
        // while(1)
        // {

        // }
        // FIXME: Origin part
        // Set Obstacle in Occupancy Grid Map of Astar
        for (int i = 0; i < obstacle_img.cols; i++)
            for (int j = 0; j < obstacle_img.rows; j++)
            {
                // Obstacle_value 값이 255*3보다 작으면, 장애물의 위치를 의미
                int obstacle_value =
                    obstacle_img.data[(i * 3 + 0) + (3 * j * obstacle_img.cols)] +
                    obstacle_img.data[(i * 3 + 1) + (3 * j * obstacle_img.cols)] +
                    obstacle_img.data[(i * 3 + 2) + (3 * j * obstacle_img.cols)];

                // Set Obstacle
                if (obstacle_value < 255 * 3)
                {
                    int x_tmp = i;
                    int y_tmp = j;
                    // h_astar.node[x_tmp + (y_tmp * occupancy_grid_map_width)].close_flag = true;

                    // Call the Closed State of Nodes
                    // int oc_x_tmp = std::round(h_astar.node[x_tmp + (y_tmp * occupancy_grid_map_width)].pos.xn);
                    // int oc_y_tmp = std::round(h_astar.node[x_tmp + (y_tmp * occupancy_grid_map_width)].pos.yn);
                    // std::cout << "x, y   : " << oc_x_tmp << "\t\t" << oc_y_tmp << std::endl;
                    // if (oc_x_tmp < 0) oc_x_tmp += h_astar.negative_val_offset;
                    // if (oc_y_tmp < 0) oc_y_tmp += h_astar.negative_val_offset;
                    // std::cout << "x, y 2 : " << oc_x_tmp << "\t\t" << oc_y_tmp << std::endl << std::endl;
                    h_astar.oc_closed_flag[i][j] = true;
                }
            }

        // // Clipping for init and final pos for existing obstacle
        // if(h_astar.node[init_pos_x + (init_pos_y * occupancy_grid_map_width)].close_flag == true |
        //         h_astar.node[init_pos_x + (init_pos_y * occupancy_grid_map_width)].close_flag == true |
        //     h_astar.node[target_x + (target_y * occupancy_grid_map_width)].close_flag == true |
        //         h_astar.node[target_x + (target_y * occupancy_grid_map_width)].close_flag == true)
        // {
        //     std::cout << "The position of Init_xy or Final_xy was occupied by obstacle. Change the pos value" << std::endl;
        //     exit(0);
        // }

        // FIXME: Origin part
        // Do Astar
        // You can get the result from vectors of astar.local_xy
        // while(!h_astar.GetResult(target_x,target_y))
        // {
        // }

        // Do Astar
        // You can get the result from vectors of astar.local_xy
        int nCount_tmp = 0;
        cv::Mat test_2(100, 100, CV_8UC3);
        test_2.setTo(255);
        cv::imwrite("../optimization_class_ws/src/hybrid_a_star_algorithm_pkg/result2.jpg", test_2);

        while (!h_astar.Run())
        {
        }

        ///////////////////////////
        ///////////////////////////
        ///////////////////////////
        ////// Visualization //////
        ///////////////////////////
        ///////////////////////////
        ///////////////////////////
        std::cout << "visualization Step 1 : Draw Visit Node ??" << std::endl;
        for (int i = 0; i < adj_nodes_x.size(); i++)
        {
            int color_tmp = i % 255;
            // cv::circle(result, cv::Point(adj_nodes_x[i], adj_nodes_y[i]), 0, cv::Scalar(0, 0, color_tmp), 1);
            // cv::circle(result, cv::Point(adj_nodes_x[i], adj_nodes_y[i]), 0, cv::Scalar(0, 0, 255), 1);
            //        cv::imshow("Result of Astar", result);
            //        cv::waitKey();
        }
        for (int i = 0; i < adj_nodes_x2.size(); i++)
        {
            // cv::circle(result, cv::Point(adj_nodes_x2[i], adj_nodes_y2[i]), 2, cv::Scalar(0, 255, 0), 1);
            // cv::circle(result, cv::Point(adj_nodes_x3[i], adj_nodes_y3[i]), 4, cv::Scalar(100, 100, 0), 1);
        }
        for (int i = 0; i < adj_nodes_x3.size(); i++)
        {
            // result.data[((int)node[i].pos.xn * 3 + 0) + (3 * (int)node[i].pos.yn * test.cols)] = 0;
            // test.data[((int)node[i].pos.xn * 3 + 1) + (3 * (int)node[i].pos.yn * test.cols)] = 0;
            // test.data[((int)node[i].pos.xn * 3 + 2) + (3 * (int)node[i].pos.yn * test.cols)] = 255;

            // cv::circle(result, cv::Point(adj_nodes_x3[i], adj_nodes_y3[i]), 4, cv::Scalar(100, 100, 0), 1);
        }

        std::cout << "visualization Step 2 : Draw Local Path" << std::endl;
        for (int i = 0; i < h_astar.local_x.size(); i++)
        {
            cv::circle(test_2, cv::Point(h_astar.local_x[i], h_astar.local_y[i]), 0, cv::Scalar(255, 0, 0), 1);
            cv::circle(result, cv::Point(h_astar.local_x[i], h_astar.local_y[i]), 3, cv::Scalar(255, 0, 0), 1);
        }
        cv::imshow("result", test_2);
        cv::imwrite("../optimization_class_ws/src/hybrid_a_star_algorithm_pkg/result2.jpg", test_2);

        std::cout << "visualization Step 3" << std::endl;
        // Occupancy Grid Map Coordinate -> OpenCV Coordinate
        // OpenCV와 Astar의 Occuapncy Grid Map의 좌표계가 다르기 때문에 상하반전 작업
        // cv::resize(obstacle_img, obstacle_img, cv::Size(960, 540));
        // cv::resize(result, result, cv::Size(960, 540));
        cv::flip(obstacle_img, obstacle_img, 0);
        // cv::flip(result, result, 0);

        //    cv::imshow("Input Obstacle Img", obstacle_img);
        cv::imshow("Result of Astar", result);
        // cv::imwrite("../optimization_class_ws/src/hybrid_a_star_algorithm_pkg/result.jpg", result);

        // If push 'q', exit the progmram
        cv::waitKey(10);
    }
    return 0;
    // //////////////////////////////
    // //////////////////////////////
    // //////////////////////////////
    // //////// Hybrid Astar ////////
    // //////////////////////////////
    // //////////////////////////////
    // //////////////////////////////
    // // Set Astar
    // // HybridAstar h_astar(occupancy_grid_map_width, occupancy_grid_map_height, init_pos_x, init_pos_y);
    // HybridAstar h_astar(target_x, target_y);
    // // h_astar.TEST();
    // // while(1)
    // // {

    // // }
    // // FIXME: Origin part
    // // Set Obstacle in Occupancy Grid Map of Astar
    // for (int i = 0; i < obstacle_img.cols; i++)
    //     for (int j = 0; j < obstacle_img.rows; j++)
    //     {
    //         // Obstacle_value 값이 255*3보다 작으면, 장애물의 위치를 의미
    //         int obstacle_value =
    //             obstacle_img.data[(i * 3 + 0) + (3 * j * obstacle_img.cols)] +
    //             obstacle_img.data[(i * 3 + 1) + (3 * j * obstacle_img.cols)] +
    //             obstacle_img.data[(i * 3 + 2) + (3 * j * obstacle_img.cols)];

    //         // Set Obstacle
    //         if (obstacle_value < 255 * 3)
    //         {
    //             int x_tmp = i;
    //             int y_tmp = j;
    //             // h_astar.node[x_tmp + (y_tmp * occupancy_grid_map_width)].close_flag = true;

    //             // Call the Closed State of Nodes
    //             // int oc_x_tmp = std::round(h_astar.node[x_tmp + (y_tmp * occupancy_grid_map_width)].pos.xn);
    //             // int oc_y_tmp = std::round(h_astar.node[x_tmp + (y_tmp * occupancy_grid_map_width)].pos.yn);
    //             // std::cout << "x, y   : " << oc_x_tmp << "\t\t" << oc_y_tmp << std::endl;
    //             // if (oc_x_tmp < 0) oc_x_tmp += h_astar.negative_val_offset;
    //             // if (oc_y_tmp < 0) oc_y_tmp += h_astar.negative_val_offset;
    //             // std::cout << "x, y 2 : " << oc_x_tmp << "\t\t" << oc_y_tmp << std::endl << std::endl;
    //             h_astar.oc_closed_flag[i][j] = true;
    //         }

    //     }

    // // // Clipping for init and final pos for existing obstacle
    // // if(h_astar.node[init_pos_x + (init_pos_y * occupancy_grid_map_width)].close_flag == true |
    // //         h_astar.node[init_pos_x + (init_pos_y * occupancy_grid_map_width)].close_flag == true |
    // //     h_astar.node[target_x + (target_y * occupancy_grid_map_width)].close_flag == true |
    // //         h_astar.node[target_x + (target_y * occupancy_grid_map_width)].close_flag == true)
    // // {
    // //     std::cout << "The position of Init_xy or Final_xy was occupied by obstacle. Change the pos value" << std::endl;
    // //     exit(0);
    // // }

    // // FIXME: Origin part
    // // Do Astar
    // // You can get the result from vectors of astar.local_xy
    // // while(!h_astar.GetResult(target_x,target_y))
    // // {
    // // }

    // // Do Astar
    // // You can get the result from vectors of astar.local_xy
    // int nCount_tmp = 0;
    // cv::Mat test_2(100, 100, CV_8UC3);
    // test_2.setTo(255);
    // cv::imwrite("../optimization_class_ws/src/hybrid_a_star_algorithm_pkg/result2.jpg", test_2);

    // while (!h_astar.Run())
    // {
    // }

    // ///////////////////////////
    // ///////////////////////////
    // ///////////////////////////
    // ////// Visualization //////
    // ///////////////////////////
    // ///////////////////////////
    // ///////////////////////////
    // std::cout << "visualization Step 1 : Draw Visit Node ??" << std::endl;
    // for (int i = 0; i < adj_nodes_x.size(); i++)
    // {
    //     int color_tmp = i % 255;
    //     // cv::circle(result, cv::Point(adj_nodes_x[i], adj_nodes_y[i]), 0, cv::Scalar(0, 0, color_tmp), 1);
    //     // cv::circle(result, cv::Point(adj_nodes_x[i], adj_nodes_y[i]), 0, cv::Scalar(0, 0, 255), 1);
    //     //        cv::imshow("Result of Astar", result);
    //     //        cv::waitKey();
    // }
    // for (int i = 0; i < adj_nodes_x2.size(); i++)
    // {
    //     // cv::circle(result, cv::Point(adj_nodes_x2[i], adj_nodes_y2[i]), 2, cv::Scalar(0, 255, 0), 1);
    //     // cv::circle(result, cv::Point(adj_nodes_x3[i], adj_nodes_y3[i]), 4, cv::Scalar(100, 100, 0), 1);
    // }
    // for (int i = 0; i < adj_nodes_x3.size(); i++)
    // {
    //     // result.data[((int)node[i].pos.xn * 3 + 0) + (3 * (int)node[i].pos.yn * test.cols)] = 0;
    //     // test.data[((int)node[i].pos.xn * 3 + 1) + (3 * (int)node[i].pos.yn * test.cols)] = 0;
    //     // test.data[((int)node[i].pos.xn * 3 + 2) + (3 * (int)node[i].pos.yn * test.cols)] = 255;

    //     // cv::circle(result, cv::Point(adj_nodes_x3[i], adj_nodes_y3[i]), 4, cv::Scalar(100, 100, 0), 1);
    // }

    // std::cout << "visualization Step 2 : Draw Local Path" << std::endl;
    // for (int i = 0; i < h_astar.local_x.size(); i++)
    // {
    //     cv::circle(test_2, cv::Point(h_astar.local_x[i], h_astar.local_y[i]), 0, cv::Scalar(255, 0, 0), 1);
    //     cv::circle(result, cv::Point(h_astar.local_x[i], h_astar.local_y[i]), 3, cv::Scalar(255, 0, 0), 1);
    // }
    // cv::imshow("result", test_2);
    // cv::imwrite("../optimization_class_ws/src/hybrid_a_star_algorithm_pkg/result2.jpg", test_2);

    // std::cout << "visualization Step 3" << std::endl;
    // // Occupancy Grid Map Coordinate -> OpenCV Coordinate
    // // OpenCV와 Astar의 Occuapncy Grid Map의 좌표계가 다르기 때문에 상하반전 작업
    // // cv::resize(obstacle_img, obstacle_img, cv::Size(960, 540));
    // // cv::resize(result, result, cv::Size(960, 540));
    // cv::flip(obstacle_img, obstacle_img, 0);
    // // cv::flip(result, result, 0);

    // //    cv::imshow("Input Obstacle Img", obstacle_img);
    // cv::imshow("Result of Astar", result);
    // // cv::imwrite("../optimization_class_ws/src/hybrid_a_star_algorithm_pkg/result.jpg", result);

    // // If push 'q', exit the progmram
    // while (cv::waitKey(10) != 'q')
    // {
    // }

    // return 0;
}
