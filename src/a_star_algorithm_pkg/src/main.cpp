#include <iostream>
#include <string>

// For Astar
#include "astar.hpp"

// For visualizer
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>



int main(int argc, char **argv)
{

    ///////////////////////////////
    ///////////////////////////////
    ///////////////////////////////
    ///////////////////////////////
    if(argc!=6)
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
    int init_pos_x = std::stoi(argv[2]) + 1;
    int init_pos_y = obstacle_img_height - 1 - std::stoi(argv[3]);

    // Set Arrival Pos_xy
    int target_x = std::stoi(argv[4]) + 1;
    int target_y = obstacle_img_height - 1 - std::stoi(argv[5]);

    // Clipping for input pos_xy
    if(init_pos_x < 3 | init_pos_y < 3 | target_x > 958 | target_y > 540)
    {
        std::cout << "The pos should be larget than (3, 3) and smaller than (958, 540)" << std::endl;
        exit(1);
    }

    ////////////////////////////////////
    ////////////////////////////////////
    ////////////////////////////////////
    //////// Set Obstacle Image ////////
    ////////////////////////////////////
    ////////////////////////////////////
    ////////////////////////////////////
    // Get Input Img from jpeg
    cv::Mat obstacle_img;
    obstacle_img = cv::imread("img/" + img_count + ".jpeg");
    // obstacle_img = cv::imread("img/" + img_count + ".png");
    if(!obstacle_img.data)
    {
        std::cout << "Can't find the image file" << std::endl;
        exit(1);
    }
    cv::resize(obstacle_img, obstacle_img, cv::Size(obstacle_img_width, obstacle_img_height));

    // OpenCV Coordinate -> Occupancy Grid Map Coordinate
    // OpenCV와 Astar의 Occuapncy Grid Map의 좌표계가 다르기 때문에 상하반전 작업
    cv::flip(obstacle_img, obstacle_img, 0);

    // Set Result Image to visualize
    cv::Mat result;
    result = obstacle_img.clone();


    ///////////////////////
    ///////////////////////
    ///////////////////////
    //////// Astar ////////
    ///////////////////////
    ///////////////////////
    ///////////////////////
    // Set Astar
    Astar astar(occupancy_grid_map_width, occupancy_grid_map_height, init_pos_x, init_pos_y);

    std::vector<std::pair<double, double>> waypoints;
    double prev_x_tmp = 0;
    for(int i = 0; i < obstacle_img.cols; i++)
        for(int j = 0; j < obstacle_img.rows; j++)
        {
            double r_tmp = obstacle_img.at<cv::Vec3b>(j, i)[2];
            double g_tmp = obstacle_img.at<cv::Vec3b>(j, i)[1];
            double b_tmp = obstacle_img.at<cv::Vec3b>(j, i)[0];
            if((r_tmp + g_tmp + b_tmp) <= 20)
            {
                if(i >= 163)
                {
                    if(i - prev_x_tmp > 5)
                    {
                        // std::cout << obstacle_img.at<cv::Vec3b>(j, i) << "\t\t" << i << "\t\t" << j << std::endl;
                        waypoints.push_back(std::make_pair(i, j));
                        // cv::circle(result, cv::Point(i, j), 0, cv::Scalar(0,0,255), 2);
                    }
                    prev_x_tmp = i;
                }
            }
        }

    // std::vector<std::pair<double, double>> obstacles;
    for(int i = 0; i < obstacle_img.cols; i++)
        for(int j = 0; j < obstacle_img.rows; j++)
        {
            double r_tmp = obstacle_img.at<cv::Vec3b>(j, i)[2];
            double g_tmp = obstacle_img.at<cv::Vec3b>(j, i)[1];
            double b_tmp = obstacle_img.at<cv::Vec3b>(j, i)[0];
            // if((r_tmp + g_tmp + b_tmp) < 700 & (r_tmp + g_tmp + b_tmp) > 500)
            // if(((r_tmp + g_tmp + b_tmp) >= 400) & ((r_tmp + g_tmp + b_tmp) < 700))
            if( b_tmp > 215 & (r_tmp + g_tmp + b_tmp < 590))
            {
                std::cout << obstacle_img.at<cv::Vec3b>(j, i) << "\t\t" << i << "\t\t" << j << std::endl;
                // obstacles.push_back(std::make_pair(i, j));
                // cv::circle(result, cv::Point(i, j), 0, cv::Scalar(0,255,0), 5);
                // astar.node[i + (j * occupancy_grid_map_width)].close_flag = true;

                for(int l = -15; l < 15; l++)
                    for(int m = -15; m<15; m++)
                    {
                        // cv::circle(result, cv::Point(i+l, j+m), 0, cv::Scalar(0,255,0), 5);
                        astar.node[i+l + ((j+m) * occupancy_grid_map_width)].close_flag = true;
                        // obstacles.push_back(std::make_pair(i, j));
                    }
            }
        }


    // Set Obstacle in Occupancy Grid Map of Astar
    // for(int i = 0; i < obstacle_img.cols; i++)
    //     for(int j = 0; j < obstacle_img.rows; j++)
    //     {
    //         // Obstacle_value 값이 255*3보다 작으면, 장애물의 위치를 의미
    //         int obstacle_value =
    //                             obstacle_img.data[(i * 3 + 0) + (3 * j * obstacle_img.cols)] +
    //                             obstacle_img.data[(i * 3 + 1) + (3 * j * obstacle_img.cols)] +
    //                             obstacle_img.data[(i * 3 + 2) + (3 * j * obstacle_img.cols)];

    //         // Set Obstacle
    //         if (obstacle_value < 255*3)
    //         {
    //             int x_tmp = i;
    //             int y_tmp = j;
    //             astar.node[x_tmp + (y_tmp * occupancy_grid_map_width)].close_flag = true;
    //         }
    //     }

    // Clipping for init and final pos for existing obstacle
    if(astar.node[init_pos_x + (init_pos_y * occupancy_grid_map_width)].close_flag == true |
            astar.node[init_pos_x + (init_pos_y * occupancy_grid_map_width)].close_flag == true |
	    astar.node[target_x + (target_y * occupancy_grid_map_width)].close_flag == true |
            astar.node[target_x + (target_y * occupancy_grid_map_width)].close_flag == true)
    {
        std::cout << "The position of Init_xy or Final_xy was occupied by obstacle. Change the pos value" << std::endl;
        exit(0);
    }

    // Do Astar
    // You can get the result from vectors of astar.local_xy
    while(!astar.GetResult(target_x,target_y))
    {
    }



    ///////////////////////////
    ///////////////////////////
    ///////////////////////////
    ////// Visualization //////
    ///////////////////////////
    ///////////////////////////
    ///////////////////////////
/*
    for(int i = 0; i < adj_nodes_x.size(); i++)
    {
        int color_tmp = i % 255;
        cv::circle(result, cv::Point(adj_nodes_x[i], adj_nodes_y[i]), 0, cv::Scalar(0, 0, color_tmp), 1);
//        cv::imshow("Result of Astar", result);
//        cv::waitKey();
    }
*/

    
    for(int i = 0; i < astar.local_x.size()-1; i++)
    {
        // cv::circle(result, cv::Point(astar.local_x[i], astar.local_y[i]), 0, cv::Scalar(0,0,255), 1);
        cv::line(result, cv::Point(astar.local_x[i], astar.local_y[i]), cv::Point(astar.local_x[i+1], astar.local_y[i+1]), cv::Scalar(0,0,255), 1, 8, 0);
    }

    // Occupancy Grid Map Coordinate -> OpenCV Coordinate
    // OpenCV와 Astar의 Occuapncy Grid Map의 좌표계가 다르기 때문에 상하반전 작업
    // cv::resize(obstacle_img, obstacle_img, cv::Size(960, 540));
    // cv::resize(result, result, cv::Size(960, 540));
    cv::flip(obstacle_img, obstacle_img, 0);
    cv::flip(result, result, 0);

//    cv::imshow("Input Obstacle Img", obstacle_img);
    cv::imshow("Result of Astar", result);
    cv::imwrite("img/result/result.jpeg", result);
    
    // If push 'q', exit the progmram
    while(cv::waitKey(10)!='q')
    {

    }


    return 0;
}
