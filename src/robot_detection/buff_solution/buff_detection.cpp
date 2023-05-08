#include "buff_detection.h"

// #define ROSBAG_DEBUG
// #define BINARY_SHOW
// #define DRAW_BUFF_CONTOURS

namespace robot_detection
{
    BuffDetector::BuffDetector()
    {
        // std::string package_path = ros::package::getPath("robot_detection");
        std::string package_path = "/home/lmx2/vision_ws_2/src/robot_detection";
        // cv::FileStorage fs(package_path + "/vision_data/buff_data.yaml", cv::FileStorage::READ);
        cv::FileStorage fs("/home/lmx2/vision_ws_2/src/robot_detection/vision_data/buff_data.yaml", cv::FileStorage::READ);

        // binary_thresh
        binary_threshold = (int)fs["binary_threshold"];   // blue 100  red  70
        // enemy_color
        buff_color = COLOR(((std::string)fs["buff_color"]));
        // std::cout<<"binary_threshold"<<binary_threshold<<std::endl;

        shoot_delay = (double)fs["shoot_delay"];

        // r
        fit_circle_counts = (int)fs["fit_circle_counts"]; 
        r_actual_distance = (double)fs["r_actual_distance"];
        error_range = (double)fs["error_range"];
        r_max_area = (double)fs["r_max_area"]; 
        r_min_area = (double)fs["r_min_area"]; 
        r_full_ratio_min = (double)fs["r_full_ratio_min"]; 
        r_full_ratio_max = (double)fs["r_full_ratio_max"]; 
        r_width_pixel = (int)fs["r_width_pixel"];
        r_height_pixel = (int)fs["r_height_pixel"];

        // buff_no
        no_buff_area_max = (double)fs["no_buff_area_max"]; 
        no_buff_area_min = (double)fs["no_buff_area_min"]; 
        no_full_ratio_min = (double)fs["no_full_ratio_min"]; 
        no_full_ratio_max = (double)fs["no_full_ratio_max"]; 

        // buff_yes
        yes_buff_area_max = (double)fs["yes_buff_area_max"]; 
        yes_buff_area_min = (double)fs["yes_buff_area_min"]; 
        yes_full_ratio_min = (double)fs["yes_full_ratio_min"]; 
        yes_full_ratio_max = (double)fs["yes_full_ratio_max"]; 

        // buff_feature
        fit_sinusoid_counts = (int)fs["fit_sinusoid_counts"];
        fit_sinusoid_time = (double)fs["fit_sinusoid_time"];

        fs.release();

        state = 0;

        symbol_scale_ratio = 0;
        radius_scale_ratio = 0;
        
        isSmallBuff = true;
        buff_type = 1;

        isClockwise = true;
        rotate_direction = 0;  //TODO: unknown to set 0
        now_angle = 0;
        rotate_speed = 0;
        const_rotate_speed = 60;    

        isInitYaw = false;

        isFindR = false;

        last_angle = 0;
        direction_count = 0;
        isFirstCalculate = false;

        isSwitch = false;
        isBegin = false;
    }

    void BuffDetector::reset()
    {
        symbol_scale_ratio = 0;
        radius_scale_ratio = 0;
        
        isSmallBuff = true;
        buff_type = 1;

        isClockwise = true;
        rotate_direction = 0;
        now_angle = 0;
        rotate_speed = 0;
        const_rotate_speed = 60;    

        isInitYaw = false;

        isFindR = false;

        last_angle = 0;
        isFirstCalculate = false;

        isSwitch = false;
        isBegin = false;
    }

    bool BuffDetector::detectResult(const cv::Mat _src, int color, chrono_time now_time)
    {
        _src.copyTo(src);
        _src.copyTo(show_src);
        buff_color = color;

        if(!isInitYaw)
        {
            isInitYaw = initYaw();
            // std::cout<<"already isInitYaw!!! -----  "<< initial_yaw[2]<<std::endl;
            return false;
        }

#ifdef ROSBAG_DEBUG
        isInitYaw = initYaw();  // TODO: for test
        // std::cout<<"already isInitYaw!!! -----  "<< initial_yaw[2]<<std::endl;
#endif

        setImage();
        extractContours();

        if(!isFindR)
        {
            if(!findRcenter())
            {
                return false;
            }
            if(!fitCircle()) 
            {
                return false;
            }
        }
        else
        {
            // 用之前清空
            components_rrt.clear();
            if(!refindRcenter())
            {
                return false;
            }
            if(!findComponents())
            {
                return false;
            }
            // 用之前清空
            buff_no.in_rrt = cv::RotatedRect();
            buff_no.out_rrt = cv::RotatedRect();
            if(!matchComponents())
            {
                return false;
            }
            calculateBuffPosition();
            // calculateScaleRatio();
            // if(!calculateRotateDirectionAndSpeed(now_time))
            // {
            //     return false;
            // }
            if(!calculateShootPositionAndAngle())
            {
                return false;
            }
        }
        return true;
    }

    void BuffDetector::show()
    {
        // show receive data on img's ru
        cv::putText(show_src,"speed : " +std::to_string(AS.bullet_speed),cv::Point2f(1280 - 300,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        cv::putText(show_src,"pitch : "+std::to_string(AS.ab_pitch),cv::Point2f(1280 - 300,90),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        cv::putText(show_src,"yaw   : "+std::to_string(AS.ab_yaw),cv::Point2f(1280 - 300,120),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        cv::putText(show_src,"roll  : "+std::to_string(AS.ab_roll),cv::Point2f(1280 - 300,150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);

        // show send data on img's lu
        cv::putText(show_src,"initial yaw: "+std::to_string(initial_yaw[2] / CV_PI * 180),cv::Point2f(0,30),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        cv::putText(show_src,"pitch : "+std::to_string(control_angle[1]),cv::Point2f(0,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        cv::putText(show_src,"yaw   : "+std::to_string(control_angle[2]),cv::Point2f(0,90),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);

        // show delta angle to rotate 
        cv::putText(show_src,"delta_p : "+std::to_string(control_angle[1] - AS.ab_pitch),cv::Point2f(0,120),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        cv::putText(show_src,"delta_y : "+std::to_string(control_angle[2] - AS.ab_yaw),cv::Point2f(0,150),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    
        cv::putText(show_src,"R pixel point",cv::Point2f(0,210),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        cv::putText(show_src,"x : "+std::to_string(r_center.pixel_position.x),cv::Point2f(0,240),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        cv::putText(show_src,"y : "+std::to_string(r_center.pixel_position.y),cv::Point2f(0,270),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);

        cv::putText(show_src,"buff pixel point",cv::Point2f(0,330),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        cv::putText(show_src,"x : "+std::to_string(AS.imu2pixel(buff_no.imu_position).x),cv::Point2f(0,360),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        cv::putText(show_src,"y : "+std::to_string(AS.imu2pixel(buff_no.imu_position).y),cv::Point2f(0,390),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);

        cv::putText(show_src,"angle : "+std::to_string(now_angle),cv::Point2f(0,450),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        cv::putText(show_src,"delta_angle : "+std::to_string(delta_angle),cv::Point2f(0,480),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        cv::putText(show_src,"delta_time : "+std::to_string(delta_time),cv::Point2f(0,510),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        cv::putText(show_src,"rotate_speed : "+std::to_string(rotate_speed),cv::Point2f(0,540),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);

        cv::putText(show_src,"R position",cv::Point2f(0,1024 - 150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(show_src,"s : "+std::to_string(r_center.buff_position.norm())+"m",cv::Point2f(0,1024 - 120),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(show_src,"x : "+std::to_string(r_center.buff_position[0]),cv::Point2f(0,1024 - 90),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(show_src,"y : "+std::to_string(r_center.buff_position[1]),cv::Point2f(0,1024 - 60),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(show_src,"z : "+std::to_string(r_center.buff_position[2]),cv::Point2f(0,1024 - 30),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        
        cv::putText(show_src,"buff position",cv::Point2f(300,1024 - 150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(show_src,"s : "+std::to_string(buff_no.buff_position.norm())+"m",cv::Point2f(300,1024 - 120),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(show_src,"x : "+std::to_string(buff_no.buff_position[0]),cv::Point2f(300,1024 - 90),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(show_src,"y : "+std::to_string(buff_no.buff_position[1]),cv::Point2f(300,1024 - 60),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(show_src,"z : "+std::to_string(buff_no.buff_position[2]),cv::Point2f(300,1024 - 30),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        
        cv::putText(show_src,"buff coordinate",cv::Point2f(600,1024 - 150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(show_src,"s : "+std::to_string(sqrt(buff_coordinate[0]*buff_coordinate[0]+buff_coordinate[2]*buff_coordinate[2]))+"m",cv::Point2f(600,1024 - 120),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(show_src,"x : "+std::to_string(buff_coordinate[0]),cv::Point2f(600,1024 - 90),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(show_src,"y : "+std::to_string(buff_coordinate[1]),cv::Point2f(600,1024 - 60),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(show_src,"z : "+std::to_string(buff_coordinate[2]),cv::Point2f(600,1024 - 30),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);

        cv::putText(show_src,"shoot position",cv::Point2f(900,1024 - 150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(show_src,"s : "+std::to_string(shoot_position.norm())+"m",cv::Point2f(900,1024 - 120),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(show_src,"x : "+std::to_string(shoot_position[0]),cv::Point2f(900,1024 - 90),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(show_src,"y : "+std::to_string(shoot_position[1]),cv::Point2f(900,1024 - 60),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(show_src,"z : "+std::to_string(shoot_position[2]),cv::Point2f(900,1024 - 30),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);

        // shoot point in image
        cv::Point2f shoot_pixel = AS.imu2pixel(AS.buff2imu(shoot_position));
        cv::circle(show_src,shoot_pixel,50,cv::Scalar(50,100,255),-1);
        // bullet point in image
        cv::Point2f bullet_pixel = AS.imu2pixel(bullet_position);
        cv::circle(show_src,bullet_pixel,5,cv::Scalar(150,100,255),-1);
        // image center
        cv::circle(show_src,cv::Point(640,512),5,cv::Scalar(255,200,100),-1);
    }

    // 转过去正对着，就可以确定yaw了，把相机和枪口的垂直中线画到图传上
    bool BuffDetector::initYaw()
    {
#ifdef ROSBAG_DEBUG
        initial_yaw[2] = 49.2 * CV_PI / 180.0;
#else
        initial_yaw[2] = -147.356 * CV_PI / 180.0;
#endif
        AS.RotationMatrix_imu2buff = AS.eulerAnglesToRotationMatrix(initial_yaw);
        return true;
    }

    void BuffDetector::setImage()
    {
        // TODO: 修改为通道相减
        // cv::Mat gray;
        // cv::cvtColor(src,gray,cv::COLOR_BGR2GRAY);
        // cv::threshold(gray,binary,binary_threshold,255,cv::THRESH_BINARY);

        std::vector<cv::Mat> channels;
        cv::split(src,channels);

#ifdef ROSBAG_DEBUG
        // rosbag debug 
        cv::threshold(channels[1],channels[1],150,255,cv::THRESH_BINARY);
        cv::threshold(channels[0],channels[0],170,255,cv::THRESH_BINARY);
        binary = channels[1] - channels[0];
#else 
        // reality
        if(buff_color == BLUE)
        {
            cv::threshold(channels[0],channels[0],170,255,cv::THRESH_BINARY);
            cv::threshold(channels[2],channels[2],254,255,cv::THRESH_BINARY);
            binary = channels[0] - channels[2];
        }
        else
        {
            cv::threshold(channels[0],channels[0],170,255,cv::THRESH_BINARY);
            cv::threshold(channels[2],channels[2],254,255,cv::THRESH_BINARY);
            binary = channels[2] - channels[0];
        }
#endif

        // // 这个二值化理论上没用
        // // cv::threshold(green_blue,binary,250,255,cv::THRESH_BINARY); 
        // // 废弃：闭操作虽然可以解决灯中间有空洞，但是也会把流水灯连起来影响检测
        // // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        // // morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

#ifdef BINARY_SHOW
        cv::imshow("b",channels[0]);
        // cv::imshow("g",channels[1]);
        cv::imshow("r",channels[2]);
        // cv::imshow("green_blue",green_blue);
        cv::imshow("binary",binary);
#endif //BINARY_SHOW
    }   

    void BuffDetector::extractContours()
    {
        // 用之前清空
        all_contours.clear();
        all_hierarchy.clear();

	    findContours(binary, all_contours, all_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
#ifdef DRAW_BUFF_CONTOURS
        for(int i=0;i< all_contours.size();i++)
            cv::drawContours(show_src,all_contours,i,cv::Scalar(255,255,0),1,cv::LINE_8);
#endif
    }

    // 工具函数：颜色判断，用于圆心和符叶组件
    bool BuffDetector::matchColor(std::vector<cv::Point> contour)
    {
        cv::RotatedRect rrt = cv::minAreaRect(contour);
        double contour_area = cv::contourArea(contour);
        cv::Rect2f rect = rrt.boundingRect();
        if (0 <= rect.x && 0 <= rect.width  && rect.x + rect.width  <= src.cols &&
            0 <= rect.y && 0 <= rect.height && rect.y + rect.height <= src.rows)
        {
            // old plan --- waste time
            // int sum_r = 0, sum_b = 0;
            // cv::Mat roi = src(rect);
            // // Iterate through the ROI
            // for (int i = 0; i < roi.rows; i++)
            // {
            //     for (int j = 0; j < roi.cols; j++)
            //     {
            //         if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) // 只加正矩形中的轮廓！！！
            //         {
            //             sum_r += roi.at<cv::Vec3b>(i, j)[2];
            //             sum_b += roi.at<cv::Vec3b>(i, j)[0];
            //         }
            //     }
            // }
            // std::cout<<sum_r<<"           "<<sum_b<<std::endl;
            // Sum of red pixels > sum of blue pixels ?
            // light.lightColor = sum_r > sum_b ? RED : BLUE;

            cv::Mat roi = src(rect);
            cv::Mat mask = binary(rect);
            cv::Scalar sum = cv::mean(roi,mask);
            // std::cout << "color: red-" << sum[2] << " | blue-" << sum[0] << " | green-" << sum[1] << std::endl;

            // if(sum_r[0]>sum_b[0])
            // {
            //     color = RED;
            // }
            // else if(sum_r[0]>sum_b[0])
            // {
            //     color = RED;
            // }
            // else if(sum_r[0]>sum_b[0])
            // {
            //     color = RED;
            // }

            int color = sum[2] > sum[0] ? RED : BLUE;

            if(color == buff_color)
                return true;
        }
        return false;
    }

    // 工具函数：重新定义旋转矩形的点，左上角顺时针
    void BuffDetector::redefineRotatedRectPoints(cv::Point2f p[], cv::RotatedRect rrt)
    {
        // 弃用，对于那种细长的旋转矩形会出bug
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

        // 产生了新的排序方案，比上面的好
        // double angle = rrt.angle * CV_PI / 180.0;
        // double a = rrt.size.width * 0.5;
        // double b = rrt.size.height * 0.5;
        // double sina = sin(angle);
        // double cosa = cos(angle);
        // cv::Point2f center = rrt.center;
        // p[0] = cv::Point2f(center.x + a * cosa - b * sina, center.y + a * sina + b * cosa);
        // p[1] = cv::Point2f(center.x - a * cosa - b * sina, center.y - a * sina + b * cosa);
        // p[2] = cv::Point2f(center.x - a * cosa + b * sina, center.y - a * sina - b * cosa);
        // p[3] = cv::Point2f(center.x + a * cosa + b * sina, center.y + a * sina - b * cosa);

        // 方案三  弃用，编译会报错
        // cv::Point2f rrt_pts[4];
	    // rrt.points(rrt_pts);
        // std::vector<std::pair<float, cv::Point2f>> polar_pts;
        // polar_pts.emplace_back(std::atan2(rrt_pts[0].y - rrt.center.y, rrt_pts[0].x - rrt.center.x), rrt_pts[0]);
        // polar_pts.emplace_back(std::atan2(rrt_pts[1].y - rrt.center.y, rrt_pts[1].x - rrt.center.x), rrt_pts[1]);
        // polar_pts.emplace_back(std::atan2(rrt_pts[2].y - rrt.center.y, rrt_pts[2].x - rrt.center.x), rrt_pts[2]);
        // polar_pts.emplace_back(std::atan2(rrt_pts[3].y - rrt.center.y, rrt_pts[3].x - rrt.center.x), rrt_pts[3]);
        // std::sort(polar_pts.begin(), polar_pts.end());
        // p[0] = polar_pts[0].second;
        // p[1] = polar_pts[1].second;
        // p[2] = polar_pts[2].second;
        // p[3] = polar_pts[3].second;

        // 弃用，编译会报错
        // cv::Point2f vertices[4];
        // rrt.points(vertices);
        // cv::Point2f center = rrt.center;
        // // 计算每个顶点相对于中心点的极角
        // std::vector<std::pair<float, cv::Point2f>> polar_pts;
        // for (int i = 0; i < 4; i++)
        // {
        //     polar_pts.emplace_back(std::atan2(vertices[i].y - center.y, vertices[i].x - center.x), vertices[i]);
        // }
        // // 按极角排序顶点
        // std::sort(polar_pts.begin(), polar_pts.end());
        // // 重新定义顶点顺序
        // p[0] = polar_pts[0].second;
        // p[1] = polar_pts[1].second;
        // p[2] = polar_pts[2].second;
        // p[3] = polar_pts[3].second;

        // 计算旋转矩形的四个顶点
        // cv::Point2f pt0, pt1, pt2, pt3;
        // rrt.points(&pt0, &pt1, &pt2, &pt3);
        // // 查找最左上角的点
        // cv::Point2f top_left_point = pt0;
        // for (int i = 1; i < 4; i++) {
        //     if (pt[i].x < top_left_point.x || (pt[i].x == top_left_point.x && pt[i].y < top_left_point.y)) {
        //         top_left_point = pt[i];
        //     }
        // }
        // // 重新排序顶点数组
        // cv::Point2f sorted_corners[4];
        // int index = 0;
        // sorted_corners[index++] = top_left_point;
        // for (int i = 0; i < 4; i++) {
        //     if (pt[i] != top_left_point) {
        //         sorted_corners[index++] = pt[i];
        //     }
        // }
        // if (sorted_corners[1].y > sorted_corners[2].y) {
        //     std::swap(sorted_corners[1], sorted_corners[2]);
        // }
        // // 将重新排序后的顶点存储到原始的顶点数组中
        // for (int i = 0; i < 4; i++) {
        //     rect_corners[i] = sorted_corners[i];
        // }
    }

    bool BuffDetector::findRcenter()
    {
        // TODO: 记得缩小ROI来寻找R标
        // std::cout<<"all_contours's size:  "<<all_contours.size()<<std::endl;
        for(int i=0; i<all_contours.size(); ++i)
        {
            double r_area = cv::contourArea(all_contours[i]);

            // RT
            cv::Rect r_rt = cv::boundingRect(all_contours[i]);
            double full_ratio = r_area / (r_rt.height * r_rt.width);
            double square_ratio = (double)r_rt.height / (double)r_rt.width;  // 不做数据类型转换看不到小数
            // RRT ------ 暂时弃用------选择RT方案，如需变回，取消注释即可
            // cv::RotatedRect r_rrt = cv::minAreaRect(all_contours[i]);
            // double full_ratio = r_area / (r_rrt.size.height * r_rrt.size.width);
            // double square_ratio = r_rrt.size.height / r_rrt.size.width;

            bool area_ok = (r_area < r_max_area) && (r_area > r_min_area) ? true : false;
            bool contour_ok = (all_hierarchy[i][2] == -1 && all_hierarchy[i][3] == -1) ? true : false;
            bool full_ratio_ok = (full_ratio < r_full_ratio_max) && (full_ratio > r_full_ratio_min) ? true : false;
            bool is_like_square = (square_ratio < 1.2) && (square_ratio > 0.8) ? true : false;
            // TODO: template R
            bool is_r_ok = true;
            contour_ok = true;
            // if(area_ok && 1 )
            // {
            //     // std::cout<<"+++"<<square_ratio<<std::endl;
            //     cv::drawContours(findR,all_contours,i,cv::Scalar(0,255,0),1,cv::LINE_8);
            // }

            if(area_ok && contour_ok && full_ratio_ok && is_like_square && is_r_ok)
            {                
                bool is_color_ok = matchColor(all_contours[i]);
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

                    // 寻找残缺不全的R的最小外接圆，通过手动把那四个PnP用的点算出来
                    cv::Point2f center;
                    float radius;
                    cv::minEnclosingCircle(all_contours[i], center, radius);
                    cv::circle(show_src,center,radius,cv::Scalar(0,255,0),1,8);
                    // std::cout<<"center point"<<center<<"   radius"<<radius<<std::endl;
                    r_center.points_4[0] = cv::Point2f(r_center.rect.x - r_width_pixel/2, r_center.rect.y - r_height_pixel/2);
                    r_center.points_4[1] = cv::Point2f(r_center.rect.x + r_width_pixel/2, r_center.rect.y - r_height_pixel/2);
                    r_center.points_4[2] = cv::Point2f(r_center.rect.x + r_width_pixel/2, r_center.rect.y + r_height_pixel/2);
                    r_center.points_4[3] = cv::Point2f(r_center.rect.x - r_width_pixel/2, r_center.rect.y + r_height_pixel/2);
                    
                    r_center.imu_position = AS.pixel2imu(r_center.points_4, BUFF_R);
                    // std::cout<<"r_center.distance      :  "<<r_center.imu_position.norm()<<std::endl;
                    r_center.buff_position = AS.imu2buff(r_center.imu_position); 
                    
                    // std::cout<<"r_center.imu_position  :  "<<r_center.imu_position.transpose()<<std::endl;
                    // std::cout<<"r_center.buff_position :  "<<r_center.buff_position.transpose()<<std::endl;


                    // 剔除不良数值，即与实际距离的值偏差大的
                    double dis = r_center.imu_position.norm();
                    if(dis < r_actual_distance + error_range && dis > r_actual_distance - error_range)
                    {
                        r_center.points_3d.emplace_back(r_center.imu_position);
                        r_center.points_2d.emplace_back(center);
                        // RT
                        r_center.pixel_position = center;
                        // RRT
                        // r_center.pixel_position = r_rrt.center;

                        return true;
                    }
                }
            }
        }
        std::cout<<"------ The target is lost in the process of fitting the circle !!!"<<std::endl;
        return false;
    }

    bool BuffDetector::fitCircle()
    {
        if(r_center.points_3d.size() <= fit_circle_counts)
        {
            // std::cout<<"r_center.points_3d.size() :  "<<r_center.points_3d.size()<<std::endl;
            // std::cout<<"r_center.points_2d.size() :  "<<r_center.points_2d.size()<<std::endl;
            return false;
        }
        else
        {
            // fit in 3d
            Eigen::Vector3d temp = r_center.points_3d[0];
            cv::Point2f temp2 = r_center.points_2d[0];
            for(int i=1; i<fit_circle_counts; ++i)
            {
                temp = (temp + r_center.points_3d[i]) / 2;
                temp2 = (temp2 + r_center.points_2d[i]) / 2;
            }
            r_center.pixel_position = temp2;
            r_center.imu_position = temp;
            r_center.buff_position = AS.imu2buff(r_center.imu_position);

            // fit in 2d
            // AS.circleLeastFit(r_center.points_2d,r_center.buff_position[0],r_center.buff_position[2],r_center.radius);

            std::cout<<"The center of the BUFF has now been fitted !!!"<<std::endl; 
            std::cout<<"BUFF R POSITION : "<<r_center.imu_position.transpose()<<std::endl;
            std::cout<<"BUFF R BUFF : "<<r_center.buff_position.transpose()<<std::endl;
            std::cout<<"BUFF R PIXEL : "<<r_center.pixel_position<<std::endl;
            std::cout<<"BUFF R DISTANCE : "<<r_center.imu_position.norm()<<std::endl;

            isFindR = true;
            // 使用完要清空
            r_center.points_3d.clear();
            r_center.points_2d.clear();
            return true;
        }
    }

    bool BuffDetector::refindRcenter()
    {
        r_center.pixel_position = AS.imu2pixel(r_center.imu_position);

        int side_length = 500;
        cv::Point2f rt_lu = cv::Point2f(r_center.pixel_position.x - side_length/2, r_center.pixel_position.y - side_length/2);      
        // rt_lu.x = rt_lu.x >= 0 ? rt_lu.x : 0;
        // rt_lu.x = rt_lu.x >= 1280 ? 1280 : rt_lu.x;
        // rt_lu.y = rt_lu.y >= 0 ? rt_lu.y : 0;
        // rt_lu.y = rt_lu.y >= 1024 ? 1024 : rt_lu.y;
        cv::Rect2f rt;
        if(rt_lu.x>=0 && rt_lu.x<=1280 && rt_lu.y>=0 && rt_lu.y<=1024)
        {
            rt = cv::Rect2f(rt_lu, cv::Size(side_length,side_length));
            cv::rectangle(show_src,rt,cv::Scalar(0,255,0),1,8);
        }
        else
        {
            return false;
        }

        for(int i=0; i<all_contours.size(); ++i)
        {
            double r_area = cv::contourArea(all_contours[i]);

            // RT
            cv::Rect r_rt = cv::boundingRect(all_contours[i]);
            double full_ratio = r_area / (r_rt.height * r_rt.width);
            double square_ratio = (double)r_rt.height / (double)r_rt.width;  // 不做数据类型转换看不到小数
            // RRT ------ 暂时弃用------选择RT方案，如需变回，取消注释即可
            // cv::RotatedRect r_rrt = cv::minAreaRect(all_contours[i]);
            // double full_ratio = r_area / (r_rrt.size.height * r_rrt.size.width);
            // double square_ratio = r_rrt.size.height / r_rrt.size.width;

            bool area_ok = (r_area < r_max_area) && (r_area > r_min_area) ? true : false;
            bool contour_ok = (all_hierarchy[i][2] == -1 && all_hierarchy[i][3] == -1) ? true : false;
            bool full_ratio_ok = (full_ratio < r_full_ratio_max) && (full_ratio > r_full_ratio_min) ? true : false;
            bool is_like_square = (square_ratio < 1.2) && (square_ratio > 0.8) ? true : false;
            bool in_roi = rt.contains(cv::Point2f(r_rt.x + r_rt.width / 2.0f, r_rt.y + r_rt.height / 2.0f));
            // TODO: template R
            bool is_r_ok = true;

            contour_ok = true;
            // if(area_ok && 1 )
            // {
            //     // std::cout<<"+++"<<square_ratio<<std::endl;
            //     cv::drawContours(findR,all_contours,i,cv::Scalar(0,255,0),1,cv::LINE_8);
            // }

            if(area_ok && contour_ok && full_ratio_ok && is_like_square && is_r_ok && in_roi)
            {                
                bool is_color_ok = matchColor(all_contours[i]);
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

                    // 寻找残缺不全的R的最小外接圆，通过手动把那四个PnP用的点算出来
                    cv::Point2f center;
                    float radius;
                    cv::minEnclosingCircle(all_contours[i], center, radius);
                    cv::circle(show_src,center,radius,cv::Scalar(0,255,0),1,8);
                    // std::cout<<"center point"<<center<<"   radius"<<radius<<std::endl;
                    r_center.points_4[0] = cv::Point2f(r_center.rect.x - r_width_pixel/2, r_center.rect.y - r_height_pixel/2);
                    r_center.points_4[1] = cv::Point2f(r_center.rect.x + r_width_pixel/2, r_center.rect.y - r_height_pixel/2);
                    r_center.points_4[2] = cv::Point2f(r_center.rect.x + r_width_pixel/2, r_center.rect.y + r_height_pixel/2);
                    r_center.points_4[3] = cv::Point2f(r_center.rect.x - r_width_pixel/2, r_center.rect.y + r_height_pixel/2);
                    
                    r_center.imu_position = AS.pixel2imu(r_center.points_4, BUFF_R);
                    // std::cout<<"r_center.distance      :  "<<r_center.imu_position.norm()<<std::endl;
                    r_center.buff_position = AS.imu2buff(r_center.imu_position); 
                    // std::cout<<"r_center.imu_position  :  "<<r_center.imu_position.transpose()<<std::endl;
                    // std::cout<<"r_center.buff_position :  "<<r_center.buff_position.transpose()<<std::endl;

                    // 剔除不良数值，即与实际距离的值偏差大的
                    double dis = r_center.imu_position.norm();
                    if(dis < r_actual_distance + error_range && dis > r_actual_distance - error_range)
                    {
                        // RT
                        r_center.pixel_position = center;
                        // RRT
                        // r_center.pixel_position = r_rrt.center;

                        return true;
                    }
                }
            }
        }
        return false;
    }

    // 找击打和未击打的符叶的零部件
    bool BuffDetector::findComponents()
    {
        for(int i=0; i<all_contours.size(); ++i)
        {
            double buff_area = cv::contourArea(all_contours[i]);
            cv::RotatedRect buff_rrt = cv::minAreaRect(all_contours[i]);
            double full_ratio = buff_area / (buff_rrt.size.height * buff_rrt.size.width);
            double w = buff_rrt.size.height > buff_rrt.size.width ? buff_rrt.size.height : buff_rrt.size.width;
            double h = buff_rrt.size.height < buff_rrt.size.width ? buff_rrt.size.height : buff_rrt.size.width;
            double rectangle_ratio = w / h;

            // buff_no
            bool no_area_ok = (buff_area < no_buff_area_max) && (buff_area > no_buff_area_min) ? true : false;
            bool no_full_ratio_ok = (full_ratio < no_full_ratio_max) && (full_ratio > no_full_ratio_min) ? true : false;
            bool no_is_like_rectangle = (rectangle_ratio < 4) && (rectangle_ratio > 2) ? true : false;          // 337/140 == 2.4  // 2.7~3.05     
            bool no_contour_ok = (all_hierarchy[i][2] == -1 && all_hierarchy[i][3] == -1) ? true : false;
            no_contour_ok = true;  // 仔细看二值图的零件中间有细小的气泡                                                                

            if(buff_area && no_full_ratio_ok && no_is_like_rectangle && no_contour_ok)
            // if(1)
            {
                // std::cout<<"rectangle_ratio : "<<buff_area<<std::endl;

                cv::drawContours(show_src,all_contours,i,cv::Scalar(0,255,0),1,cv::LINE_8);

                bool is_color_ok = matchColor(all_contours[i]);
                if(is_color_ok)
                    components_rrt.emplace_back(buff_rrt);
            }

            // // // TODO: 讨论识别它的必要性，多加一点拟合用的数据
            // // // buff_yes
            // // bool yes_area_ok = (buff_area < yes_buff_area_max) && (buff_area > yes_buff_area_min) ? true : false;
            // // bool yes_full_ratio_ok = (full_ratio < yes_full_ratio_max) && (full_ratio > yes_full_ratio_min) ? true : false;
            // // bool yes_is_like_rectangle = (rectangle_ratio < 1.3) && (rectangle_ratio > 1.2) ? true : false;
            // // //TODO: find aim contour 
            // // bool yes_contour_ok = (hierarchy[i][2] == -1 && hierarchy[i][3] == -1) ? true : false;
            // // if(yes_area_ok && yes_full_ratio_ok && yes_is_like_rectangle && yes_contour_ok)
            // // {
            // //     return true;
            // // }
        }

        // std::cout<<"components_rrt.size  : "<<components_rrt.size()<<std::endl;
        for(int i=0; i<components_rrt.size(); ++i)
        {
            cv::Point2f buff_rrt_pts[4];
            components_rrt[i].points(buff_rrt_pts);
            for (int i = 0; i < 4; ++i) {
                line(show_src, buff_rrt_pts[i], buff_rrt_pts[(i + 1) % 4], CV_RGB(180, 0, 255),1,cv::LINE_8);
            }
        }
        
        if(components_rrt.size() < 2)
        {
            // 保存未识别到两个零件的图像
            // error_cnt++;
            // std::string path = "/home/lmx2/error_pic0/" + std::to_string(error_cnt) + ".jpg";
            // cv::imwrite(path,binary);

            // 曾经有13帧丢失，发现仔细看二值图的零件中间有细小的气泡   
            std::cout<<"no 2 components !!!  -----  "<< components_rrt.size()<<std::endl;  
            return false;
        }
        else
        {
            return true;
        }
    }

    // 匹配找到的零部件，靠三点一线约束，在图像坐标系上操作
    bool BuffDetector::matchComponents()
    {
        // 圆心在每一帧内的像素坐标点
        // r_center.pixel_position = AS.imu2pixel(r_center.imu_position);

        cv::circle(show_src,r_center.pixel_position,5,cv::Scalar(0,255,0),-1);

        for(int i = 0; i < components_rrt.size(); ++i)
        {
            for(int j = 0; j < components_rrt.size(); ++j)
            {
                if(i == j)
                    continue;

                double radius1 = POINT_DIST(r_center.pixel_position,components_rrt[i].center);  // out 0.8160m
                double radius2 = POINT_DIST(r_center.pixel_position,components_rrt[j].center);  // in  0.5805m
                double radius_ratio = radius1 / radius2;

                // 1.6  1.3
                bool radius1_ratio_ok = (radius_ratio < 2) && (radius_ratio > 1) ? true : false;
                bool radius2_ratio_ok = (radius_ratio < 1) && (radius_ratio > 0.5) ? true : false;
                double one_line_angle = AS.pointsInLine(r_center.pixel_position,components_rrt[j].center,components_rrt[i].center,2);
                bool in_one_line =  one_line_angle < 5 ? true : false;

                if((radius1_ratio_ok || radius2_ratio_ok) && in_one_line)
                {
                    if(radius1_ratio_ok)
                    {
                        buff_no.in_rrt = components_rrt[j];
                        buff_no.out_rrt = components_rrt[i];
                    }
                    if(radius2_ratio_ok)
                    {
                        buff_no.in_rrt = components_rrt[i];
                        buff_no.out_rrt = components_rrt[j];
                    }

                    buff_no.pixel_position = (components_rrt[j].center + components_rrt[i].center) / 2;

                    // cv::circle(show_src,buff_no.pixel_position,5,cv::Scalar(0,255,0),-1);
                    
                    // double angle_offset = fabs(atan2(components_rrt[j].center.y - r_center.pixel_position.y, components_rrt[j].center.x - r_center.pixel_position.x)/CV_PI*180.0 - atan2(components_rrt[i].center.y - r_center.pixel_position.y, components_rrt[i].center.x - r_center.pixel_position.x)/CV_PI*180.0); 
                    // std::cout<<"data:  "<<angle_offset<<"   "<<in_one_line<<std::endl;
                    // std::cout<<"A non-hit BUFF was detected !!!"<<std::endl;
                    return true;
                }
            }
        }

        // for debug to save error picture
        // is_need_to_save = true;

        // double angle_offset = fabs(atan2(components_rrt[j].center.y - r_center.pixel_position.y, components_rrt[j].center.x - r_center.pixel_position.x)/CV_PI*180.0 - atan2(components_rrt[i].center.y - r_center.pixel_position.y, components_rrt[i].center.x - r_center.pixel_position.x)/CV_PI*180.0); 
        // std::cout<<"data:  "<<atan2(components_rrt[j].center.y - r_center.pixel_position.y, components_rrt[j].center.x - r_center.pixel_position.x)/CV_PI*180.0<<"   "<<atan2(components_rrt[i].center.y - r_center.pixel_position.y, components_rrt[i].center.x - r_center.pixel_position.x)/CV_PI*180.0<<std::endl;
        
        std::cout   <<"No non-hit BUFF found !!!  ---  "
                    // <<radius_ratio<<"  " <<one_line_angle
                    <<std::endl;      
        return false;
    }

    // 重点在于五点检测的输入点的顺序
    bool BuffDetector::calculateBuffPosition()
    {
        cv::Point2f out_rrt_pts[4];
	    buff_no.out_rrt.points(out_rrt_pts);
        cv::Point2f in_rrt_pts[4];
	    buff_no.in_rrt.points(in_rrt_pts);

        // // 通过约束来确定，不管输入顺序是否有影响（已经弃用）
        // // out_rrt
        // int s1 = INT_MAX, s2 = INT_MAX; // s1存储最小值，s2存储第二小值
        // int s_idx1 = 0, s_idx2 = 1;
        // for (int i = 0; i < 4; ++i)
        // {
        //     double dis = POINT_DIST(out_rrt_pts[i],r_center.pixel_position);
        //     if (dis < s1)
        //     {
        //         s2 = s1;
        //         s1 = dis;
        //         s_idx1 = i;
        //     }
        //     else if (dis < s2)
        //     {
        //         s2 = dis;
        //         s_idx2 = i;
        //     }
        // }
        // // in_rrt
        // int b1 = 0, b2 = 0; // b1存储最大值，b2存储第二大值
        // int b_idx1 = 0, b_idx2 = 1;
        // for (int i = 0; i < 4; ++i)
        // {
        //     double dis = POINT_DIST(in_rrt_pts[i],r_center.pixel_position);
        //     if (dis > b1)
        //     {
        //         b2 = b1;
        //         b1 = dis;
        //         b_idx1 = i;
        //     }
        //     else if (dis > b2)
        //     {
        //         b2 = dis;
        //         b_idx2 = i;
        //     }
        // }
        // buff_no.points_5[0] = out_rrt_pts[0];
        // buff_no.points_5[1] = out_rrt_pts[1];
        // buff_no.points_5[2] = in_rrt_pts[1];
        // buff_no.points_5[3] = in_rrt_pts[0];
        // buff_no.points_5[4] = r_center.pixel_position;

        // 通过规定顺序来确定五点
        // redefineRotatedRectPoints(out_rrt_pts,buff_no.out_rrt);
        // redefineRotatedRectPoints(in_rrt_pts,buff_no.in_rrt);
        // 展示经过规定后的零件的旋转矩阵的点
        cv::circle(show_src,out_rrt_pts[0],2,cv::Scalar(255,0,0),-1);
        cv::circle(show_src,out_rrt_pts[1],2,cv::Scalar(0,255,0),-1);
        cv::circle(show_src,out_rrt_pts[2],2,cv::Scalar(0,0,255),-1);
        cv::circle(show_src,in_rrt_pts[0],2,cv::Scalar(255,0,0),-1);
        cv::circle(show_src,in_rrt_pts[1],2,cv::Scalar(0,255,0),-1);
        cv::circle(show_src,in_rrt_pts[2],2,cv::Scalar(0,0,255),-1);

        // buff坐标系下规定，从x正方向旋转一周为0~360，用这个计算角度差值是有问题的  
        double angle = atan2((buff_no.pixel_position.y - r_center.pixel_position.y),(buff_no.pixel_position.x - r_center.pixel_position.x));
        angle = -angle / CV_PI * 180;
        if(angle<0)
            angle += 360;
        now_angle = angle;
        // last_angle = now_angle;  // behind 

        // // 确定五点的输入顺序   (redefineRotatedRectPoints + this)
        // if(angle>45 && angle<=135)
        // {
        //     buff_no.points_5[0] = out_rrt_pts[3];
        //     buff_no.points_5[1] = out_rrt_pts[2];
        //     buff_no.points_5[2] = in_rrt_pts[1];
        //     buff_no.points_5[3] = r_center.pixel_position;
        //     buff_no.points_5[4] = in_rrt_pts[0];
        // }
        // else if(angle>135 && angle<=225)
        // {
        //     buff_no.points_5[0] = out_rrt_pts[2];
        //     buff_no.points_5[1] = out_rrt_pts[1];
        //     buff_no.points_5[2] = in_rrt_pts[0];
        //     buff_no.points_5[3] = r_center.pixel_position;
        //     buff_no.points_5[4] = in_rrt_pts[3];
        // }
        // else if(angle>225 && angle<=315)
        // {
        //     buff_no.points_5[0] = out_rrt_pts[1];
        //     buff_no.points_5[1] = out_rrt_pts[0];
        //     buff_no.points_5[2] = in_rrt_pts[3];
        //     buff_no.points_5[3] = r_center.pixel_position;
        //     buff_no.points_5[4] = in_rrt_pts[2];
        // }
        // else
        // {
        //     buff_no.points_5[0] = out_rrt_pts[0];
        //     buff_no.points_5[1] = out_rrt_pts[3];
        //     buff_no.points_5[2] = in_rrt_pts[2];
        //     buff_no.points_5[3] = r_center.pixel_position;
        //     buff_no.points_5[4] = in_rrt_pts[1];
        // }

        // 确定五点的输入顺序   (origin，默认使用官方的旋转矩形的角点的顺序) (好用，只需要删除个别异常值)
        // 引入旋转矩形的宽高来做额外限制 (plus)
        double out_h = buff_no.out_rrt.size.height;
        double out_w = buff_no.out_rrt.size.width;
        double in_h = buff_no.in_rrt.size.height;
        double in_w = buff_no.in_rrt.size.width;
        if(angle>0 && angle<90 && out_h>out_w && in_h>in_w)
        {
            buff_no.points_5[0] = out_rrt_pts[1];
            buff_no.points_5[1] = out_rrt_pts[0];
            buff_no.points_5[2] = in_rrt_pts[3];
            buff_no.points_5[3] = r_center.pixel_position;
            buff_no.points_5[4] = in_rrt_pts[2];
        }
        else if(angle>90 && angle<180 && out_h<out_w && in_h<in_w)
        {
            buff_no.points_5[0] = out_rrt_pts[0];
            buff_no.points_5[1] = out_rrt_pts[3];
            buff_no.points_5[2] = in_rrt_pts[2];
            buff_no.points_5[3] = r_center.pixel_position;
            buff_no.points_5[4] = in_rrt_pts[1];
        }
        else if(angle>180 && angle<270 && out_h>out_w && in_h>in_w)
        {
            buff_no.points_5[0] = out_rrt_pts[3];
            buff_no.points_5[1] = out_rrt_pts[2];
            buff_no.points_5[2] = in_rrt_pts[1];
            buff_no.points_5[3] = r_center.pixel_position;
            buff_no.points_5[4] = in_rrt_pts[0];
        }
        else if(angle>270 && angle<360 && out_h<out_w && in_h<in_w)
        {
            buff_no.points_5[0] = out_rrt_pts[2];
            buff_no.points_5[1] = out_rrt_pts[1];
            buff_no.points_5[2] = in_rrt_pts[0];
            buff_no.points_5[3] = r_center.pixel_position;
            buff_no.points_5[4] = in_rrt_pts[3];
        }

        if((angle>355&&angle<360) || (angle>=0&&angle<5) || (angle>85&&angle<95) || (angle>175&&angle<185) || (angle>265&&angle<275) )
        {
            // 通过规定顺序来确定五点
            redefineRotatedRectPoints(out_rrt_pts,buff_no.out_rrt);
            redefineRotatedRectPoints(in_rrt_pts,buff_no.in_rrt);
            if(angle>45 && angle<=135)
            {
                buff_no.points_5[0] = out_rrt_pts[3];
                buff_no.points_5[1] = out_rrt_pts[2];
                buff_no.points_5[2] = in_rrt_pts[1];
                buff_no.points_5[3] = r_center.pixel_position;
                buff_no.points_5[4] = in_rrt_pts[0];
            }
            else if(angle>135 && angle<=225)
            {
                buff_no.points_5[0] = out_rrt_pts[2];
                buff_no.points_5[1] = out_rrt_pts[1];
                buff_no.points_5[2] = in_rrt_pts[0];
                buff_no.points_5[3] = r_center.pixel_position;
                buff_no.points_5[4] = in_rrt_pts[3];
            }
            else if(angle>225 && angle<=315)
            {
                buff_no.points_5[0] = out_rrt_pts[1];
                buff_no.points_5[1] = out_rrt_pts[0];
                buff_no.points_5[2] = in_rrt_pts[3];
                buff_no.points_5[3] = r_center.pixel_position;
                buff_no.points_5[4] = in_rrt_pts[2];
            }
            else
            {
                buff_no.points_5[0] = out_rrt_pts[0];
                buff_no.points_5[1] = out_rrt_pts[3];
                buff_no.points_5[2] = in_rrt_pts[2];
                buff_no.points_5[3] = r_center.pixel_position;
                buff_no.points_5[4] = in_rrt_pts[1];
            }
        }

        // 绘制未击打的符叶
        for (int m = 0; m < 5; ++m)
        {
            line(show_src, buff_no.points_5[m], buff_no.points_5[(m + 1) % 5], CV_RGB(0, 255, 0),2,cv::LINE_8);
        } 
        // // 绘制重新定义后的PnP的输入点
        // cv::circle(show_src,buff_no.points_5[0],3,cv::Scalar(255,0,0),-1);
        // cv::circle(show_src,buff_no.points_5[1],2,cv::Scalar(0,255,0),-1);
        // cv::circle(show_src,buff_no.points_5[2],2,cv::Scalar(0,0,255),-1);
        // cv::circle(show_src,buff_no.points_5[3],3,cv::Scalar(255,255,0),-1);
        // cv::circle(show_src,buff_no.points_5[4],2,cv::Scalar(255,0,0),-1);
        // // 绘制符叶角度和括号的旋转矩形的边长在左上角
        // cv::putText(show_src,std::to_string(angle),cv::Point2f(0,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        // std::string out = std::to_string(buff_no.out_rrt.angle) + "   " + std::to_string(buff_no.out_rrt.size.width) + "   " + std::to_string(buff_no.out_rrt.size.height);
        // cv::putText(show_src,out,cv::Point2f(0,90),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        // std::string in = std::to_string(buff_no.in_rrt.angle) + "   " + std::to_string(buff_no.in_rrt.size.width) + "   " + std::to_string(buff_no.in_rrt.size.height);
        // cv::putText(show_src,in,cv::Point2f(0,120),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);

        // 使用转换的点
        cv::circle(show_src,AS.imu2pixel(buff_no.imu_position),2,cv::Scalar(0,255,0),-1);
        // cv::circle(show_src,buff_no.pixel_position,2,cv::Scalar(0,255,0),-1);

        // // 未击打大符坐标计算
        buff_no.imu_position = AS.pixel2imu(buff_no.points_5, BUFF_NO);
        // std::cout<<"buff_no.imu_position  : "<< buff_no.imu_position.transpose() <<"  &  dis : "<<buff_no.imu_position.norm()<<std::endl;
        buff_no.buff_position = AS.imu2buff(buff_no.imu_position);
        // std::cout<<"buff_no.buff_position : "<< buff_no.buff_position.transpose()<<"  &  dis : "<<buff_no.imu_position.norm()<<std::endl;

        buff_coordinate = {buff_no.buff_position[0]-r_center.buff_position[0],buff_no.buff_position[1]-r_center.buff_position[1],buff_no.buff_position[2]-r_center.buff_position[2]};
        // std::cout<<"buff_coordinate : "<< buff_coordinate.transpose()<<"   radius: "<<buff_coordinate.norm()<< std::endl;

        // if(buff_no.imu_position.norm()<2)
        // {
        //     error_cnt++;
        //     std::string path = "/home/lmx2/error_pic2/" + std::to_string(error_cnt) + ".jpg";
        //     cv::imwrite(path,show_src);
        // }

        for(int i=0; i<5; ++i)
            buff_no.points_5[i] = cv::Point2f();
        return true;
    }

    // 当前测距和实际距离的缩放比例，依靠识别到的R来确定    (弃用)
    bool BuffDetector::calculateScaleRatio()
    {
        double actual_symbol_diagonal_distance = sqrt(AS.buff_r_h*AS.buff_r_h + AS.buff_r_w*AS.buff_r_w);
        // TODO: 做好坐标系转换
        // Eigen::Vector3d tl = AS.pixel2imu()
        double compute_symbol_diagonal_distance = 1;
        symbol_scale_ratio = actual_symbol_diagonal_distance / compute_symbol_diagonal_distance;

        double actual_radius_length = AS.buff_radius;
        double compute_radius_length = (buff_no.buff_position - r_center.buff_position).norm();
        radius_scale_ratio = actual_radius_length / compute_radius_length;
        
        return true;
    }

    // (弃用)
    bool BuffDetector::isSwitchBuff()
    {
        
    }

    // 最简单的就是让操作手输入旋转方向，不行就自己计算旋转方向，靠向量的叉乘计算    采用余弦定理可以计算帧与帧之间的旋转角度
    bool BuffDetector::calculateRotateDirectionAndSpeed(chrono_time now_time)
    {
        // 向量的叉乘计算
        Eigen::Vector3d now_vector;
        if(!isFirstCalculate)
        {
            // last_vector = buff_no.buff_position - r_center.buff_position;
            last_time = std::chrono::high_resolution_clock::now();
            last_angle = now_angle;

            isFirstCalculate = true;
            return false;
        }
        else
        {
            now_vector = buff_no.buff_position - r_center.buff_position;
            // TODO: 这里用了三个坐标轴的值
            // double numerator = last_vector.norm()*last_vector.norm() + now_vector.norm()*now_vector.norm() - (now_vector - last_vector).norm()*(now_vector - last_vector).norm();
            // double denominator = 2 * last_vector.norm() * now_vector.norm();
            // double cos_delta_angle =  numerator / denominator;
            // // TODO: 防止出现无穷大数据，因为asin函数如果输入参大于1就会出现无穷大数据
            // delta_angle = acos(cos_delta_angle) * 180.0 / CV_PI;
            
            // delta_angle
            // 上面的弃用，采用图像坐标系计算角度
            delta_angle = now_angle - last_angle;
            if(delta_angle > 36)
            {
                // 这属于是切换符叶了
                std::cout<<"switch buff !!!"<<std::endl;
                return false;
            }
            
            if(delta_angle > 0)
            {
                direction_count++;
            }
            else
            {
                direction_count--;
            }

            if(direction_count > 10)
            {
                isClockwise = true;
                direction_count = 0;
            }
            else if(direction_count < -10)
            {
                isClockwise = false;
                direction_count = 0;
            }
            else
            {
                return false;
            }
            // calculate rotate direction
            // isClockwise = last_vector[0]*now_vector[2]-last_vector[2]*now_vector[0] > 0 ? false : true;
            rotate_direction = isClockwise ? -1 : 1;

            // delta_time
            delta_time = seconds_duration(now_time - last_time).count();

            // calculate rotate speed
            rotate_speed = fabs((delta_angle * CV_PI / 180) / delta_time);
            // 保证速度在实际范围内 
            if(rotate_speed<0)
                rotate_speed = 0;   // 余弦定理算的角度肯定没有负值，为了好看才加这一段代码
            else if(rotate_speed>2.09)
                rotate_speed = 2.09;
            speed_vector.emplace_back(rotate_speed);

            // last_vector = now_vector;
            last_time = now_time;
            last_angle = now_angle;

            if(!isBegin)
            {
                begin_time = std::chrono::high_resolution_clock::now();
                isBegin = true;
            }

            // std::cout<<"cos_delta_angle  : "<<cos_delta_angle<<std::endl;
            // std::cout<<"delta_angle  : "<<delta_angle<<std::endl;
            // std::cout<<"delta_time   : "<<delta_time<<std::endl;
            // std::cout<<"rotate_speed : "<<rotate_speed<<std::endl;


            if(isClockwise)
            {
                // cv::putText(show_src,"CW",cv::Point2f(0,210),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3); 
            }
            else
            {
                // cv::putText(show_src,"C-CW",cv::Point2f(0,210),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
            }
            return true;
        }
    }

    bool BuffDetector::fitSinusoid()
    {
        chrono_time now_time = std::chrono::high_resolution_clock::now();
        double delta_time = seconds_duration(now_time - last_time).count();
        if(delta_time <= fit_sinusoid_time)
        {
            return false;
        }
        else
        {
            double all_speed_sum = 0;
            for(int i=0; i<speed_vector.size(); ++i)
            {
                all_speed_sum += speed_vector[i];
            }
            b = all_speed_sum / speed_vector.size();
            a = 2.090 - b;
            // TODO: 这个 w 不好算啊 --- 算周期，波峰减去波谷的时间差
            w = asin((rotate_speed - b) / a) / delta_time;

            return true;
        }

    }

    // 工具函数： 维护角度在0~360之间
    double BuffDetector::maintainAngleLegal(double angle)
    {
        if(angle < 0)
        {
            angle += 360;
        }
        else if(angle > 360)
        {
            angle -= 360;
        }
        return angle;
    }

    bool BuffDetector::calculateShootPositionAndAngle()
    {
        buff_type = CONST_SPEED;
        rotate_direction = 1;
        if(buff_type == CONST_SPEED)
        {
            // calculate time
            double delta_t = AS.getFlyTime(buff_no.imu_position) + shoot_delay; 
            double delta_angle = delta_t * const_rotate_speed;
            aim_angle = now_angle + rotate_direction * delta_angle;
            

            // bullet_position = 
        }
        else if(buff_type == VARIABLE_SPEED)
        {

        }
        aim_angle = maintainAngleLegal(aim_angle);

        shoot_position[0] = buff_no.buff_position[0] + cos(aim_angle) * AS.buff_radius;
        shoot_position[1] = buff_no.buff_position[1];
        shoot_position[2] = buff_no.buff_position[2] + sin(aim_angle) * AS.buff_radius;

        control_angle = AS.getAngle(AS.buff2imu(shoot_position),bullet_position);

        // control_angle = AS.getAngle(buff_no.imu_position,bullet_position);
        return true;
    }   
}