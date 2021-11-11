#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "circle_judge.hpp"
#include <iostream>

int main()
{
    pcl::PointCloud<pcl::PointXY> input_point;
    pcl::PointXY point1 = {1,0};
    pcl::PointXY point2 = {-1,0};
    pcl::PointXY point3 = {0,1};
    pcl::PointXY point4 = {0,-1};
    pcl::PointXY point5 = {-1,-3};
    input_point.push_back(point1);
    input_point.push_back(point2);
    input_point.push_back(point3);
    input_point.push_back(point4);
    input_point.push_back(point5);
    
    std::cout<<"now points are :";
    std::cout<<std::endl;
    for(auto p:input_point)
    {
        std::cout<<p.x<<"   "<<p.y<<std::endl;
    }
    std::cout<<std::endl;

    circle_judge test(input_point);
    double judge = test.gradient_descent();
    std::cout<<"rate is "<<judge<<std::endl;
    return 1;
}