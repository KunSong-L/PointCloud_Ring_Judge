#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/common/distances.h>
#include <Eigen/Dense>
#include <math.h>
#include <limits>

class circle_judge
{
    public:
        circle_judge(pcl::PointCloud<pcl::PointXY> input_point, bool information_flag = false):need_information(information_flag)
        {
            std::cout<<"initializing............"<<std::endl;
            double sum_x = 0,sum_y =0;
            size = input_point.size();
            point = Eigen::MatrixXf::Ones(2,size);
            int count = 0;
            for(auto p:input_point)
            {
                sum_x += p.x;
                sum_y += p.y;
                point(0,count) = p.x;
                point(1,count) = p.y;
                count ++;
            }

            center(0) = sum_x/size;
            center(1) = sum_y/size;

            R = 0;
            for(int i =0;i<size;i++)
            {
                R += pow((point.col(i)-center).array().square().sum(),0.5);
            }
            R = R/size;
            std::cout<<"the guess center of these point is:  "<<center(0)<<"  "<<center(1)<<std::endl;
            std::cout<<"the guess  radius  of these point is:  "<<R<<std::endl;
            std::cout<<"initialize end  "<<std::endl;
            
        }

        Eigen::Vector3f gradient_caulate()
        {
            Eigen::Matrix<float,2,Eigen::Dynamic > point_minus_center;
            point_minus_center = point;
            Eigen::Matrix<float,2,Eigen::Dynamic > squared_point_minus_center;
            squared_point_minus_center = point;
            
            Eigen::Vector3f gradient;
            for(int i =0;i<size;i++)
            {
                point_minus_center.col(i) -= center;
                squared_point_minus_center.col(i) = point_minus_center.col(i).array().square();
                gradient(0) += -4*(squared_point_minus_center.col(i).sum()-R)*point_minus_center(0,i);
                gradient(1) += -4*(squared_point_minus_center.col(i).sum()-R)*point_minus_center(1,i);
                gradient(2) += -2*(squared_point_minus_center.col(i).sum()-R);
            }
            gradient(0) = gradient(0)/size;
            gradient(1) = gradient(1)/size;
            gradient(2) = gradient(2)/size;

            return gradient;
        }


        double error(Eigen::Vector2f now_center,double now_R)
        {
            double error =0;
            Eigen::Matrix<float,2,Eigen::Dynamic> point_minus_center;
            point_minus_center = point;
            Eigen::Matrix<float,2,Eigen::Dynamic> squared_point_minus_center;
            squared_point_minus_center = point;

            for(int i =0;i<size;i++)
            {
                point_minus_center.col(i) -= now_center;
                squared_point_minus_center.col(i) = point_minus_center.col(i).array().square();
                error += pow(squared_point_minus_center.col(i).sum()-now_R,2);
            }
            return error/size;
            
        }

        double gradient_descent()
        {
            int max_step = 100;
            double gradient_tol = 0.1;
            for(int j=0;j<max_step;j++)
            {
                double step_size = 2;
                Eigen::Vector3f gradient = gradient_caulate();
                if(need_information)
                {
                    std::cout<<"now center is "<<center(0)<<"  "<<center(1)<<"  ||  now R is "<<R<<
                "  ||  now gradient is "<<gradient(0)<<" "<<gradient(1)<<" "<<gradient(2)<<std::endl;
                }
                
                Eigen::Vector2f next_center;
                double next_R,next_error;
                next_center(0) = center(0) -step_size*gradient(0);
                next_center(1) = center(1) -step_size*gradient(1);
                next_R = R - step_size*gradient(2);

                // choose right step size
                
                for(int i =1;i<100;i++)
                {
                    next_error = error(next_center,next_R);
                    if (next_error<now_error)
                    {
                        now_error = next_error;
                        center = next_center;
                        R = next_R;
                        break;
                    }
                    step_size = step_size*0.8;
                    next_center(0) = center(0) -step_size*gradient(0);
                    next_center(1) = center(1) -step_size*gradient(1);
                    next_R = R - step_size*gradient(2);
                }

                if(gradient.norm()<gradient_tol) break;
            }

            return now_error/pow(R,2); //equal to zero means this is a circle
            


        }
    
    private:
        Eigen::MatrixXf point;
        Eigen::Vector2f center;
        double R;
        int size;
        double now_error = std::numeric_limits<double>::max();
        bool need_information = false;
};