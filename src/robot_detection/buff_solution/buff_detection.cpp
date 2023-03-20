#include "buff_detection.h"

namespace robot_detection
{
    BuffDetector::BuffDetector()
    {
        cv::FileStorage fs("/home/lmx2/HJ_SENTRY_VISION/src/robot_detection/vision_data/detect_data.yaml", cv::FileStorage::READ);

        //binary_thresh
        binary_threshold = (int)fs["binary_threshold"];   // blue 100  red  70
        //enemy_color
        enemy_color = COLOR(((std::string)fs["enemy_color"]));

        // R
        fit_circle_counts = (int)fs["fit_circle_counts"]; 
        r_max_area = (double)fs["r_max_area"]; 
        r_min_area = (double)fs["r_min_area"]; 
        r_full_ratio_min = (double)fs["r_full_ratio_min"]; 
        r_full_ratio_max = (double)fs["r_full_ratio_max"]; 

        // Buff_no
        no_buff_area_max = (double)fs["no_buff_area_max"]; 
        no_buff_area_min = (double)fs["no_buff_area_min"]; 
        no_full_ratio_min = (double)fs["no_full_ratio_min"]; 
        no_full_ratio_max = (double)fs["no_full_ratio_max"]; 

        // Buff_yes
        yes_buff_area_max = (double)fs["yes_buff_area_max"]; 
        yes_buff_area_min = (double)fs["yes_buff_area_min"]; 
        yes_full_ratio_min = (double)fs["yes_full_ratio_min"]; 
        yes_full_ratio_max = (double)fs["yes_full_ratio_max"]; 


        fs.release();
    }

    bool BuffDetector::detectRsult(const cv::Mat src)
    {
        setImage(src);
        extractContours();

        if(!isFindR)
        {
            if(!findRcenter())
                return false;
            if(!fitCircle())
                return false;
        }



        return true;
    }

    void BuffDetector::setImage(const cv::Mat src)
    {
        src.copyTo(_src);

        //二值化
        cv::Mat gray;
        cvtColor(_src,gray,cv::COLOR_BGR2GRAY);
        threshold(gray,_binary,binary_threshold,255,cv::THRESH_BINARY);
#ifdef BINARY_SHOW
        imshow("_binary",_binary);
#endif //BINARY_SHOW
    } 

    void BuffDetector::extractContours()
    {
	    findContours(_binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    }       

    bool BuffDetector::findRcenter()
    {
        for(int i=0; i<contours.size(); ++i)
        {
            double r_area = cv::contourArea(contours[i]);
            cv::Rect2f r_rt = cv::boundingRect(contours[i]);
            double full_ratio = r_area / (r_rt.height * r_rt.width);
            double square_ratio = r_rt.height / r_rt.width;

            bool area_ok = (r_area < r_max_area) && (r_area > r_min_area) ? true : false;
            bool contour_ok = (hierarchy[i][2] == -1 && hierarchy[i][3] == -1) ? true : false;
            bool full_ratio_ok = (full_ratio < r_full_ratio_max) && (full_ratio > r_full_ratio_min) ? true : false;
            bool is_like_square = (square_ratio < 1.2) && (square_ratio > 0.8) ? true : false;
            // TODO: template R
            bool is_r_ok;  
            if(area_ok && contour_ok && full_ratio_ok && is_r_ok)
            {
                r_center.r_rect = r_rt;
                r_center.r_rect_points = {
                    {r_center.r_rect.x, r_center.r_rect.y},
                    {r_center.r_rect.x + r_center.r_rect.width, r_center.r_rect.y},
                    {r_center.r_rect.x + r_center.r_rect.width, r_center.r_rect.y + r_center.r_rect.height},
                    {r_center.r_rect.x, r_center.r_rect.y + r_center.r_rect.height},
                };
                r_center.r_center_imu_position = AS.pixel2imu(r_center.r_rect_points, BUFF_R);
                r_center.r_center_points.push_back(r_center.r_center_imu_position);
                return true;
            }
        }
        return false;
    }

    bool BuffDetector::fitCircle()
    {
        // TODO: fit circle, 检测不良数值
        if(r_center.r_center_points.size() <= fit_circle_counts)
        {
            return false;
        }
        else
        {
            Eigen::Vector3d temp = r_center.r_center_points[0];
            for(int i=1; i<fit_circle_counts; ++i)
            {
                temp = (temp + r_center.r_center_points[i]) / 2;
            }
            r_center.r_center_imu_position = temp;
            isFindR = true;
            return true;
        }
    }

    // 找击打和未击打的符叶的零部件
    bool BuffDetector::findComponents()
    {
        for(int i=0; i<contours.size(); ++i)
        {
            double buff_area = cv::contourArea(contours[i]);
            cv::RotatedRect buff_rrt = cv::minAreaRect(contours[i]);
            double full_ratio = buff_area / (buff_rrt.size.height * buff_rrt.size.width);
            double w = buff_rrt.size.height > buff_rrt.size.width ? buff_rrt.size.height : buff_rrt.size.width;
            double h = buff_rrt.size.height < buff_rrt.size.width ? buff_rrt.size.height : buff_rrt.size.width;
            double rectangle_ratio = w / h;

            // buff_no
            bool no_area_ok = (buff_area < no_buff_area_max) && (buff_area > no_buff_area_min) ? true : false;
            bool no_full_ratio_ok = (full_ratio < no_full_ratio_max) && (full_ratio > no_full_ratio_min) ? true : false;
            bool no_is_like_rectangle = (rectangle_ratio < 2.5) && (rectangle_ratio > 2.2) ? true : false;
            bool no_contour_ok = (hierarchy[i][2] == -1 && hierarchy[i][3] == -1) ? true : false;
            if(no_area_ok && no_full_ratio_ok && no_is_like_rectangle && no_contour_ok)
            {
                components_rrt.push_back(buff_rrt);
            }

            // TODO: 讨论识别它的必要性，多加一点拟合用的数据
            // buff_yes
            // bool yes_area_ok = (buff_area < yes_buff_area_max) && (buff_area > yes_buff_area_min) ? true : false;
            // bool yes_full_ratio_ok = (full_ratio < yes_full_ratio_max) && (full_ratio > yes_full_ratio_min) ? true : false;
            // bool yes_is_like_rectangle = (rectangle_ratio < 1.3) && (rectangle_ratio > 1.2) ? true : false;
            // //TODO: find aim contour 
            // bool yes_contour_ok = (hierarchy[i][2] == -1 && hierarchy[i][3] == -1) ? true : false;
            // if(yes_area_ok && yes_full_ratio_ok && yes_is_like_rectangle && yes_contour_ok)
            // {
            //     return true;
            // }
        }

        if(components_rrt.empty())
            return false;
        else
            return true;
    }

    // 匹配找到的零部件，靠三点一线约束
    bool BuffDetector::matchComponents()
    {
        cv::Point2f r_point = AS.imu2pixel(r_center.r_center_imu_position);

        for(int i = 0; i < components_rrt.size(); ++i)
        {
            for(int j = 0; j < components_rrt.size(); ++j)
            {
                if(i == j)
                    continue;

                double radius1 = POINT_DIST(r_point,components_rrt[i].center);  // out 0.8160m
                double radius2 = POINT_DIST(r_point,components_rrt[j].center);  // in  0.5805m
                double radius_ratio = radius1 / radius2;

                bool radius_ratio_ok = (radius_ratio < 1.5) && (radius_ratio > 1.3) ? true : false;
                bool in_one_line = AS.pointsInLine(r_point,components_rrt[j].center,components_rrt[i].center,10,0);
                if(radius_ratio_ok && in_one_line)
                {
                    buff_no.in_rrt = components_rrt[j];
                    buff_no.out_rrt = components_rrt[i];
                }
            }
        }
        return true;
    }


}