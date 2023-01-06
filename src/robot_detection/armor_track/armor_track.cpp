#include "armor_track.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#define DRAW_CENTER_CIRCLE
#define DRAW_MATCH_ARMOR
#define DRAW_BULLET_POINT

namespace robot_detection {

ArmorTracker::ArmorTracker() {

        cv::FileStorage fs("/home/lmx2/vision_ws_2/src/robot_detection/vision_data/track_data.yaml", cv::FileStorage::READ);

        KF.initial_KF();

        locate_target = false;
        enemy_armor = Armor();

        tracker_state = MISSING;
        tracking_id = 0;

        find_aim_cnt = 0;
        find_threshold = (int)fs["find_threshold"];

        lost_aim_cnt = 0;
        lost_threshold = (int)fs["lost_threshold"];

        new_old_threshold = (double)fs["new_old_threshold"];

        shoot_delay = (double)fs["shoot_delay"];

        isChangeSameID = false;
        fs.release();
    }

    void ArmorTracker::reset()
    {
        t=-1;
        pitch = 0;
        yaw = 0;

        KF.initial_KF();
        Singer.Reset();

        locate_target = false;
        enemy_armor = Armor();
        tracker_state = MISSING;
        tracking_id = 0;
        find_aim_cnt = 0;
        lost_aim_cnt = 0;
    }

    // count IoU
    double ArmorTracker::countArmorIoU(Armor armor1, Armor armor2)
    {
        double area1 = armor1.size.area();
        double area2 = armor2.size.area();

        std::vector<cv::Point2f> cross_points;
        cv::rotatedRectangleIntersection(armor1, armor2, cross_points);

        double area3 = cv::contourArea(cross_points);

        return (area3) / (area1 + area2 - area3);
    }

    // 初始化，选择最优装甲板，设置卡尔曼的F和x_1，当没有目标时返回false，选一个最优目标返回true
    bool ArmorTracker::initial(std::vector<Armor> find_armors)
    {
        if(find_armors.empty())
        {
            return false;
        }

        Armor best_armor;
        int maxGrade = 0;
        for(auto & armor : find_armors)
        {
            if (armor.grade > maxGrade)
            {
                maxGrade = armor.grade;
                best_armor = armor;
            }
        }

        // select enemy
        enemy_armor = best_armor;
        tracker_state = DETECTING;
        tracking_id = enemy_armor.id;

        // initial KF --- x_post
        KF.initial_KF();
        enemy_armor.imu_position = AS.pixel2imu(enemy_armor,1);
        KF.setXPost(enemy_armor.imu_position);
//        std::cout<<"track_initial!!"<<std::endl;
        return true;
    }

    // dt是两帧之间时间间隔, 跟得住目标
    bool ArmorTracker::selectEnemy2(std::vector<Armor> find_armors, double dt)
    {
        KF.setF(dt);
        predicted_enemy = KF.predict();

#ifdef DRAW_MATCH_ARMOR
        cv::Mat m_a = _src.clone();
        // after update 上一帧预测得出本帧的位置
        cv::Point2f q = AS.imu2pixel(predicted_enemy.head(3));
        cv::circle(m_a,q,enemy_armor.size.width/10,cv::Scalar(145,14,148),-1);
#endif

        Armor matched_armor;
        bool matched = false;

        if(!find_armors.empty())
        {
            double min_position_diff = DBL_MAX;
            for(auto & armor : find_armors)
            {
                armor.imu_position = AS.pixel2imu(armor,1);
                Eigen::Vector3d pre = predicted_enemy.head(3);
                double position_diff = (pre - armor.imu_position).norm();

                if (position_diff < min_position_diff)
                {
                    min_position_diff = position_diff;
                    matched_armor = armor;
                }
            }
            std::cout<<"min_position_diff(m):    "<<min_position_diff<<std::endl;
            std::cout<<"match_id: "<<matched_armor.id<<std::endl;
            // 这个相同id对遮挡情况似乎不好
            if (min_position_diff < new_old_threshold && matched_armor.id == tracking_id)
            {
                std::cout<<"yes"<<std::endl;
                matched = true;
                Eigen::Vector3d position_vec = AS.pixel2imu(matched_armor,1);
                predicted_enemy = KF.update(position_vec);
            }
            else
            {
                std::cout<<"no"<<std::endl;
                // 本帧内是否有相同ID
                for (auto & armor : find_armors)
                {
                    if (armor.id == tracking_id)
                    {
                        matched = true;
                        KF.initial_KF();
                        // 下面这句在上面做过，不需要
//                        armor.imu_position = AS.pixel2imu(armor,1);
                        Eigen::VectorXd position_speed(6);
                        position_speed << armor.imu_position, predicted_enemy.tail(3);
                        KF.setPosAndSpeed(armor.imu_position,predicted_enemy.tail(3));
                        predicted_enemy = position_speed;
                        matched_armor = armor;

                        std::cout<<"track_sameID_initial!!"<<std::endl;
                        break;
                    }
                }
            }

        }

        if (matched)
        {

#ifdef DRAW_MATCH_ARMOR
            // matched armor center  青色
            cv::circle(m_a,matched_armor.center,matched_armor.size.width/15,cv::Scalar(255,255,0),-1);
            // after update  黄色
            cv::Point2f p = AS.imu2pixel(predicted_enemy.head(3));
            cv::circle(m_a,p,matched_armor.size.width/20,cv::Scalar(0,255,255),-1);
            cv::imshow("DRAW_MATCH_ARMOR",m_a);
#endif

            std::cout<<"enemy_armor_cam: "<<matched_armor.camera_position.transpose()<<std::endl;
            std::cout<<"enemy_armor_imu: "<<matched_armor.imu_position.transpose()<<std::endl;

            std::cout<<"predicted_enemy: "<<predicted_enemy.transpose()<<std::endl;
            std::cout<<"P: \n"<<KF.P<<std::endl;
            enemy_armor = matched_armor;
        }
//        predicted_position = predicted_enemy.head(3);
//        predicted_speed = predicted_enemy.tail(3);

        if (tracker_state == DETECTING)
        {
            // DETECTING
            if (matched)
            {
                find_aim_cnt++;
                if (find_aim_cnt > find_threshold)
                {
                    find_aim_cnt = 0;
                    tracker_state = TRACKING;
                }
            }
            else
            {
                find_aim_cnt = 0;
                tracker_state = MISSING;
            }

        }
        else if (tracker_state == TRACKING)
        {
            if (!matched)
            {
                tracker_state = LOSING;
                lost_aim_cnt++;
            }

        }
        else if (tracker_state == LOSING)
        {
            if (!matched)
            {
                lost_aim_cnt++;
                if (lost_aim_cnt > lost_threshold)
                {
                    lost_aim_cnt = 0;
                    tracker_state = MISSING;
                }
            }
            else
            {
                tracker_state = TRACKING;
                lost_aim_cnt = 0;
            }
        }

        if (tracker_state == MISSING)
        {
            reset();
            return false;
        }

        KF.setPosAndSpeed(enemy_armor.imu_position,predicted_enemy.tail(3));
        if(tracker_state == LOSING)
        {
            enemy_armor.imu_position = predicted_enemy.head(3);
            KF.setPosAndSpeed(enemy_armor.imu_position,predicted_enemy.tail(3));
        }

        return true;
    }

    // 对处于跟踪和正在丢失状态时做 预测，引入各种时间
    bool ArmorTracker::estimateEnemy(double dt)
    {
        // 对跟踪状态和正在丢失状态时做预测
        if(tracker_state == TRACKING || tracker_state == LOSING)
        {
#ifdef DRAW_BULLET_POINT
            cv::Mat bullet = _src.clone();
#endif
            // enemy_armor get real information to predicted by singer
            Eigen::Vector3d pre_pos_imu = predicted_enemy.head(3);
            circle(bullet,enemy_armor.center,5,cv::Scalar(255,0,0),-1);

            Eigen::Matrix<double,2,1> measure(enemy_armor.imu_position(0,0),enemy_armor.imu_position(1,0));
//        std::cout<<"measurex:"<<measure[0]<<std::endl;
//        std::cout<<"measurey:"<<measure[1]<<std::endl;
            double all_time = shoot_delay + AS.getFlyTime(enemy_armor.imu_position);
            ////////////////Singer predictor//////////////////////////////
            Singer.PredictInit(dt);
//            std::cout<<"predict_front:"<<Singer.predict(false)<<std::endl;
//            std::cout<<"correct:"<<Singer.correct(measure)<<std::endl;
            Singer.PredictInit(all_time);
            Eigen::Matrix<double,6,1> predicted_result = Singer.predict(true);
//            std::cout<<"result:"<<predicted_result<<std::endl;
            predicted_position << predicted_result(0,0),predicted_result(3,0),enemy_armor.imu_position(2,0);

//            Eigen::Vector3d pre_pos_cam = AS.imu2cam(predicted_position);
            bullet_point = AS.airResistanceSolve(pre_pos_imu);
            cv::Point2f bullet_drop = AS.imu2pixel(bullet_point);
            std::cout<<"pixelPos"<<bullet_drop<<std::endl;
#ifdef DRAW_BULLET_POINT
            cv::circle(bullet,bullet_drop,enemy_armor.size.width/15,cv::Scalar(127,255,0),-1);
            cv::imshow("DRAW_BULLET_POINT",bullet);
//            if(tracker_state == LOSING) cv::waitKey(0);
#endif
            return true;
        }
        else if (tracker_state == DETECTING)
        {
            enemy_armor.imu_position = AS.pixel2imu(enemy_armor,1);
            circle(_src,enemy_armor.center,5,cv::Scalar(255,0,0),-1);
            Eigen::Matrix<double,2,1> measure(enemy_armor.imu_position(0,0),enemy_armor.imu_position(1,0));
//            std::cout<<"measurex:"<<measure[0]<<std::endl;
//            std::cout<<"measurey:"<<measure[1]<<std::endl;
            double all_time = shoot_delay + AS.getFlyTime(enemy_armor.imu_position);
            ////////////////Singer predictor//////////////////////////////
            Singer.PredictInit(dt);
            /*std::cout<<"predict_front:"<<*/Singer.predict(false)/*<<std::endl*/;
            /*std::cout<<"correct:"<<*/Singer.correct(measure)/*<<std::endl*/;
            Singer.PredictInit(all_time);
            Eigen::Matrix<double,6,1> predicted_result = Singer.predict(true);
//            std::cout<<"result:"<<predicted_result<<std::endl;
            predicted_position << predicted_result(0,0),predicted_result(3,0),enemy_armor.imu_position(2,0);
            bullet_point = AS.airResistanceSolve(enemy_armor.imu_position);
            cv::Point pixelPos = AS.imu2pixel(bullet_point);
            //        std::cout<<"pixelPos"<<pixelPos<<std::endl;
            circle(_src,pixelPos,5,cv::Scalar(0,0,255),-1);
            cv::imshow("_src",_src);
//            cv::waitKey(0);
            ////////////////Singer predictor//////////////////////////////
            locate_target = false;
            return false;
        }
        else
        {
            locate_target = false;
            return false;
        }

    }


    // 返回false保持现状，返回true开始控制    ddt --- chrono
    bool ArmorTracker::locateEnemy(cv::Mat src, std::vector<Armor> armors, double time)
    {
        _src = src;
        AS._src = src;

        if(!locate_target)
        {
            if(initial(armors))
            {
                locate_target = true;
                return true;
            }
            else
            {
                locate_target = false;
                return false;
            }
        }
        else
        {

//            Eigen::Vector3d qwe = AS.pnpSolve(armors[0].armor_pt4,armors[0].type,1);


            if (t == -1)
            {
                t = time;
                return false;
            }
            double dt = (time - t) / (double)cv::getTickFrequency();
            std::cout<<"dt:"<<dt<<std::endl;
            t = time;

            if(!selectEnemy2(armors,dt))
            {
                return false;
            }

            if(!estimateEnemy(dt))
            {
                return false;
            }

            Eigen::Vector3d head_angle = AS.yawPitchSolve(bullet_point);

            pitch = head_angle[1];
            yaw = head_angle[2];

            std::cout<<"pitch: "<<pitch<<std::endl;
            std::cout<<"yaw  : "<<yaw  <<std::endl;
//            std::cout<<"KF_P"<<KF.P<<std::endl;

            return true;
        }

    }

}