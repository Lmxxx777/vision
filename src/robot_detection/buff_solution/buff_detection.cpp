#include "buff_detection.h"

#define BINARY_SHOW
#define DRAW_BUFF_CONTOURS

namespace robot_detection
{
    BuffDetector::BuffDetector()
    {
        state = 0;

        isSmallBuff = false;
        isClockwise = false;
        isInitYaw = false;
        isFirstCalculate = false;

        cv::FileStorage fs("/home/lmx2/HJ_SENTRY_VISION/src/robot_detection/vision_data/detect_data.yaml", cv::FileStorage::READ);

        //binary_thresh
        binary_threshold = (int)fs["binary_threshold"];   // blue 100  red  70
        //enemy_color
        buff_color = COLOR(((std::string)fs["buff_color"]));

        // R
        fit_circle_counts = (int)fs["fit_circle_counts"]; 
        r_actual_distance = (double)fs["r_actual_distance"];
        error_range = (double)fs["error_range"];
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

        // TODO: need to define buff_color 
        buff_color = BLUE;
    }

    void BuffDetector::reset()
    {

    }

    bool BuffDetector::detectRsult(const cv::Mat src)
    {
        if(!isInitYaw)
        {
            isInitYaw = initYaw();
            return false;
        }

        setImage(src);
        extractContours();

        if(!isFindR)
        {
            if(!findRcenter())
                return false;
            if(!fitCircle())
                return false;
            if(!calculateScaleRatio())
                return false;
        }
        else
        {
            if(findComponents())
            {
                matchComponents();
            }
            if(!calculateBuffPosition())
            {
                return false;
            }
            if(!calculateRotateDirectionAndSpeed())
            {
                return false;
            }

            
        }

        return true;
    }

    // 转过去正对着，就可以确定yaw了，把相机和枪口的垂直中线画到图传上
    bool BuffDetector::initYaw()
    {
        Eigen::Vector3d yaw = {0, 0, AS.ab_yaw};
        AS.RotationMatrix_imu2buff = AS.eulerAnglesToRotationMatrix(yaw);
        return true;
    }

    void BuffDetector::setImage(const cv::Mat src)
    {
        src.copyTo(_src);

        cv::Mat gray;
        cv::cvtColor(_src,gray,cv::COLOR_BGR2GRAY);
        cv::threshold(gray,_binary,binary_threshold,255,cv::THRESH_BINARY);
#ifdef BINARY_SHOW
        cv::imshow("_binary",_binary);
#endif //BINARY_SHOW
    } 

    void BuffDetector::extractContours()
    {
	    findContours(_binary, all_contours, all_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
#ifdef DRAW_BUFF_CONTOURS
        cv::Mat buff_contour_src;
        _src.copyTo(buff_contour_src);
        for(int i=0;i< all_contours.size();i++)
            cv::drawContours(buff_contour_src,all_contours,i,cv::Scalar(0,255,255),2,cv::LINE_8);
        imshow("DRAW_BUFF_CONTOURS",buff_contour_src);
#endif
    }      

    // 工具函数：颜色判断，用于圆心和符叶组件
    bool BuffDetector::matchColor(std::vector<cv::Point2f> contour)
    {
        cv::RotatedRect rrt = cv::minAreaRect(contour);
        double contour_area = cv::contourArea(contour);
        cv::Rect2f rect = rrt.boundingRect();
        if (0 <= rect.x && 0 <= rect.width  && rect.x + rect.width  <= _src.cols &&
            0 <= rect.y && 0 <= rect.height && rect.y + rect.height <= _src.rows)
        {
            int sum_r = 0, sum_b = 0;
            cv::Mat roi = _src(rect);
            for (int i = 0; i < roi.rows; i++)
            {
                for (int j = 0; j < roi.cols; j++)
                {
                    if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) // 只加正矩形中的轮廓！！！
                    {
                        sum_r += roi.at<cv::Vec3b>(i, j)[2];
                        sum_b += roi.at<cv::Vec3b>(i, j)[0];
                    }
                }
            }
            int color = sum_r > sum_b ? RED : BLUE;
            if(color = buff_color)
                return true;
        }
        return false;
    }

    // 工具函数：重新定义旋转矩形的点，左上角顺时针
    void BuffDetector::redefineRotatedRectPoints(cv::Point2f p[], cv::RotatedRect rrt)
    {
        rrt.points(p);
        cv::Point2f temp;
        //按X轴排序
        std::sort(p, p+4,[](cv::Point2f a, cv::Point2f b) {return a.x < b.x; });
        // X坐标前两个定义左上,左下
        if (p[0].y > p[1].y) {
            temp = p[0];
            p[0] = p[1];
            p[1] = p[3];
            p[3] = temp;
        }
        else {
            temp = p[3];
            p[3] = p[1];
            p[1] = temp;
        }
        // X坐标后两个定义右上右下角
        if (p[1].y > p[2].y) {
            temp = p[1];
            p[1] = p[2];
            p[2] = temp;
        }
    }

    bool BuffDetector::findRcenter()
    {
        for(int i=0; i<r_contours.size(); ++i)
        {
            double r_area = cv::contourArea(r_contours[i]);

            //RT
            cv::Rect r_rt = cv::boundingRect(r_contours[i]);
            double full_ratio = r_area / (r_rt.height * r_rt.width);
            double square_ratio = r_rt.height / r_rt.width;
            // RRT ------ 暂时  弃用  ，选择RT方案，如需变回，取消注释即可
            // cv::RotatedRect r_rrt = cv::minAreaRect(r_contours[i]);
            // double full_ratio = r_area / (r_rrt.size.height * r_rrt.size.width);
            // double square_ratio = r_rrt.size.height / r_rrt.size.width;

            bool area_ok = (r_area < r_max_area) && (r_area > r_min_area) ? true : false;
            bool contour_ok = (r_hierarchy[i][2] == -1 && r_hierarchy[i][3] == -1) ? true : false;
            bool full_ratio_ok = (full_ratio < r_full_ratio_max) && (full_ratio > r_full_ratio_min) ? true : false;
            bool is_like_square = (square_ratio < 1.2) && (square_ratio > 0.8) ? true : false;
            // TODO: template R
            bool is_r_ok;  
            if(area_ok && contour_ok && full_ratio_ok && is_like_square && is_r_ok)
            {
                bool is_color_ok = matchColor(r_contours[i]);
                if(is_color_ok)
                {
                    // RT
                    r_center.rect = r_rt;
                    r_center.points_4[0] = r_center.rect.tl();
                    r_center.points_4[1] = cv::Point2f(r_center.rect.x + r_center.rect.width, r_center.rect.y);
                    r_center.points_4[2] = r_center.rect.br();
                    r_center.points_4[3] = cv::Point2f(r_center.rect.x, r_center.rect.y + r_center.rect.height);
                    // RRT
                    // r_center.rotatedrect = r_rrt;
                    // cv::Point2f pts[4];
                    // redefineRotatedRectPoints(pts,r_rrt);
                    // for(int index = 0; index < 4; index++)
                    //     r_center.points_4[index] = pts[index];


                    r_center.imu_position = AS.pixel2imu(r_center.points_4, BUFF_R); 
                    r_center.buff_position = AS.imu2buff(r_center.imu_position); 

                    // 剔除不良数值，即与实际距离的值偏差大的
                    double dis = r_center.buff_position.norm();
                    if(dis < r_actual_distance + error_range && dis > r_actual_distance - error_range)
                    {
                        r_center.points_3d.emplace_back(r_center.buff_position);
                        r_center.points_2d.emplace_back(cv::Point2f(r_center.buff_position[0],r_center.buff_position[2]));
                        // RT
                        r_center.pixel_position = cv::Point2f(r_center.rect.x + r_center.rect.width/2, r_center.rect.y + r_center.rect.height/2);
                        // RRT
                        // r_center.pixel_position = r_rrt.center;
                        return true;
                    }
                }
            }
        }
        return false;
    }

    bool BuffDetector::fitCircle()
    {
        if(r_center.points_3d.size() <= fit_circle_counts)
        {
            return false;
        }
        else
        {
            // fit in 3d
            Eigen::Vector3d temp = r_center.points_3d[0];
            for(int i=1; i<fit_circle_counts; ++i)
            {
                temp = (temp + r_center.points_3d[i]) / 2;
            }
            r_center.buff_position = temp;

            // fit in 2d
            AS.circleLeastFit(r_center.points_2d,r_center.buff_position[0],r_center.buff_position[2],r_center.radius);

            isFindR = true;
            return true;
        }
    }

    // 当前测距和实际距离的缩放比例，依靠识别到的R来确定
    bool BuffDetector::calculateScaleRatio()
    {
        double actual_diagonal_distance = sqrt(AS.buff_r_h*AS.buff_r_h + AS.buff_r_w*AS.buff_r_w);
        // TODO: 做好坐标系转换
        // Eigen::Vector3d tl = AS.pixel2imu()
        double compute_diagonal_distance = 1;

        scale_ratio = actual_diagonal_distance / compute_diagonal_distance;
        return true;
    }

    // 找击打和未击打的符叶的零部件
    bool BuffDetector::findComponents()
    {
        for(int i=0; i<buff_contours.size(); ++i)
        {
            double buff_area = cv::contourArea(buff_contours[i]);
            cv::RotatedRect buff_rrt = cv::minAreaRect(buff_contours[i]);
            double full_ratio = buff_area / (buff_rrt.size.height * buff_rrt.size.width);
            double w = buff_rrt.size.height > buff_rrt.size.width ? buff_rrt.size.height : buff_rrt.size.width;
            double h = buff_rrt.size.height < buff_rrt.size.width ? buff_rrt.size.height : buff_rrt.size.width;
            double rectangle_ratio = w / h;

            // buff_no
            bool no_area_ok = (buff_area < no_buff_area_max) && (buff_area > no_buff_area_min) ? true : false;
            bool no_full_ratio_ok = (full_ratio < no_full_ratio_max) && (full_ratio > no_full_ratio_min) ? true : false;
            bool no_is_like_rectangle = (rectangle_ratio < 2.5) && (rectangle_ratio > 2.2) ? true : false;
            bool no_contour_ok = (buff_hierarchy[i][2] == -1 && buff_hierarchy[i][3] == -1) ? true : false;
            if(no_area_ok && no_full_ratio_ok && no_is_like_rectangle && no_contour_ok)
            {
                bool is_color_ok = matchColor(r_contours[i]);
                if(is_color_ok)
                    components_rrt.emplace_back(buff_rrt);
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

        if(components_rrt.size() < 2)
        {
            std::cerr<<"no 2 components !!!"<<std::endl;
            return false;
        }
        else
        {
            return true;
        }
    }

    // 匹配找到的零部件，靠三点一线约束
    bool BuffDetector::matchComponents()
    {
        // 圆心在每一帧内的像素坐标点
        r_center.pixel_position = AS.imu2pixel(r_center.imu_position);

        for(int i = 0; i < components_rrt.size(); ++i)
        {
            for(int j = 0; j < components_rrt.size(); ++j)
            {
                if(i == j)
                    continue;

                double radius1 = POINT_DIST(r_center.pixel_position,components_rrt[i].center);  // out 0.8160m
                double radius2 = POINT_DIST(r_center.pixel_position,components_rrt[j].center);  // in  0.5805m
                double radius_ratio = radius1 / radius2;

                bool radius_ratio_ok = (radius_ratio < 1.5) && (radius_ratio > 1.3) ? true : false;
                bool in_one_line = AS.pointsInLine(r_center.pixel_position,components_rrt[j].center,components_rrt[i].center,10,0);
                if(radius_ratio_ok && in_one_line)
                {
                    buff_no.in_rrt = components_rrt[j];
                    buff_no.out_rrt = components_rrt[i];
                    buff_no.pixel_position = (components_rrt[j].center + components_rrt[i].center) / 2;
                }
            }
        }
        return true;
    }

    // 重点在于五点检测的输入点的顺序
    bool BuffDetector::calculateBuffPosition()
    {
        cv::Point2f out_rrt_pts[4];
	    buff_no.out_rrt.points(out_rrt_pts);
        cv::Point2f in_rrt_pts[4];
	    buff_no.in_rrt.points(in_rrt_pts);

        // TODO: 评估和实测下面两种方法的效果，理论上来说后一个会好一点，但是怕有特殊情况

        // 通过约束来确定，不管输入顺序是否有影响
        // out_rrt
        int s1 = INT_MAX, s2 = INT_MAX; // s1存储最小值，s2存储第二小值
        int s_idx1 = 0, s_idx2 = 1;
        for (int i = 0; i < 4; ++i)
        {
            double dis = POINT_DIST(out_rrt_pts[i],r_center.pixel_position);
            if (dis < s1)
            {
                s2 = s1;
                s1 = dis;
                s_idx1 = i;
            }
            else if (dis < s2)
            {
                s2 = dis;
                s_idx2 = i;
            }
        }
        // in_rrt
        int b1 = 0, b2 = 0; // b1存储最大值，b2存储第二大值
        int b_idx1 = 0, b_idx2 = 1;
        for (int i = 0; i < 4; ++i)
        {
            double dis = POINT_DIST(in_rrt_pts[i],r_center.pixel_position);
            if (dis > b1)
            {
                b2 = b1;
                b1 = dis;
                b_idx1 = i;
            }
            else if (dis > b2)
            {
                b2 = dis;
                b_idx2 = i;
            }
        }

        buff_no.points_5[0] = out_rrt_pts[0];
        buff_no.points_5[1] = out_rrt_pts[1];
        buff_no.points_5[2] = in_rrt_pts[1];
        buff_no.points_5[3] = in_rrt_pts[0];
        buff_no.points_5[4] = r_center.pixel_position;

        // 通过规定顺序来确定五点
        redefineRotatedRectPoints(out_rrt_pts,buff_no.out_rrt);
        redefineRotatedRectPoints(in_rrt_pts,buff_no.in_rrt);
        buff_no.points_5[0] = out_rrt_pts[0];
        buff_no.points_5[1] = out_rrt_pts[1];
        buff_no.points_5[2] = in_rrt_pts[1];
        buff_no.points_5[3] = in_rrt_pts[0];
        buff_no.points_5[4] = r_center.pixel_position;

        // 未击打大符坐标计算
        buff_no.imu_position = AS.pixel2imu(buff_no.points_5, BUFF_NO);
        buff_no.buff_position = AS.imu2buff(buff_no.imu_position);

        return true;
    }

    // 最简单的就是让操作手输入旋转方向，不行就自己计算旋转方向，靠向量的叉乘计算    采用余弦定理可以计算帧与帧之间的旋转角度
    bool BuffDetector::calculateRotateDirectionAndSpeed()
    {
        // TODO: 有待改进
        // 在buff坐标系下规定，从x正方向旋转一周为0~360，用这个计算角度差值是有问题的
        double angle = atan2((buff_no.buff_position[2] - r_center.buff_position[2]),(buff_no.buff_position[0] - r_center.buff_position[0]));
        angle = angle / CV_PI * 180;
        if(angle<0)
            angle += 360;
        
        if(!isFirstCalculate)
        {
            last_angle = angle;
        }
        else
        {
            if(angle-last_angle<0)
            {
                isClockwise = false;
                last_angle = angle;
            }
            else
            {
                isClockwise = true;
                last_angle = angle;
            }
        }

        // 向量的叉乘计算
        Eigen::Vector3d now_vector;
        if(!isFirstCalculate)
        {
            last_vector = buff_no.buff_position - r_center.buff_position;
        }
        else
        {
            now_vector = buff_no.buff_position - r_center.buff_position;


            last_vector = now_vector;
        }

        double numerator = last_vector.norm()*last_vector.norm() + now_vector.norm()*now_vector.norm() - (now_vector - last_vector).norm()*(now_vector - last_vector).norm();
        double denominator = 2 * last_vector.norm() * now_vector.norm();
        double cos_delta_angle =  numerator / denominator;
        double delta_angle = acos(cos_delta_angle) * 180.0 / CV_PI;

        
    }
    
    
}