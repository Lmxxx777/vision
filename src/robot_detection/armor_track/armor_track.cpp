#include "armor_track.h"
// #include <opencv2/calib3d.hpp>
// #include <opencv2/core/eigen.hpp>

#define ANTI_SPIN
// #define PRE

namespace robot_detection {

    ArmorTracker::ArmorTracker() {

        // std::string package_path = ros::package::getPath("robot_detection");
        std::string package_path = "/home/lmx2/vision_ws_2/src/robot_detection";
        cv::FileStorage fs(package_path  + "/vision_data/track_data.yaml", cv::FileStorage::READ);

        KF.initial_KF();

        locate_target = false;
        wait_start = true;
        enemy_armor = Armor();

        is_aim_virtual_armor = false;

        tracker_state = MISSING;
        tracking_id = -1;

        find_aim_cnt = 0;
        find_threshold = (int)fs["find_threshold"];

        lost_aim_cnt = 0;
        lost_threshold = (int)fs["lost_threshold"];

        new_old_threshold = (double)fs["new_old_threshold"];

        switch_armor_cnt = 0;
        switch_armor_threshold = (int)fs["switch_armor_threshold"];

        is_switch_time_set = false;
        switch_time_threshold = (double)fs["switch_time_threshold"];

        //switch_enemy_cnt = 0;
        switch_enemy_threshold = (int)fs["switch_enemy_threshold"];
        max_effective_distance = (double)fs["max_effective_distance"];
        vir_max = 20;
        vir_num = 0;
        last_r = 0.3;
        anti_spin_max_r_multiple = (double)fs["anti_spin_max_r_multiple"];
        anti_spin_judge_low_thres = (int)fs["anti_spin_judge_low_thres"];
        anti_spin_judge_high_thres = (int)fs["anti_spin_judge_high_thres"];
        max_delta_t = (int)fs["max_delta_t"];
        max_history_len = (int)fs["max_history_len"];
///-----------------------------switchEnemy-----------------
        memset(switch_enemy_cnt, 0, sizeof switch_enemy_cnt);
        memset(flag, false, sizeof flag);
        memset(temp, 0, sizeof temp);
        delta_distance = (double)fs["delta_distance"];
        hero_distance= (double)fs["hero_distance"];
//-----------------------------------------------------------        
        fs.release();
    }

    void ArmorTracker::reset()
    {
        wait_start = true;

        KF.initial_KF();
        Singer.Reset();

        locate_target = false;
        enemy_armor = Armor();
        tracker_state = MISSING;
        tracking_id = -1;
        find_aim_cnt = 0;
        lost_aim_cnt = 0;
        //switch_enemy_cnt = 0;
    }

    // 初始化，选择最优装甲板，设置卡尔曼的F和x_1，当没有目标时返回false，选一个最优目标返回true
    bool ArmorTracker::initial(std::vector<Armor> find_armors)
    {
        if(find_armors.empty())
        {
            // std::cout<<"no track enemy!"<<std::endl;
            return false;
        }

        // select enemy : for sentry to find nearest aim
        double distance = DBL_MAX;
        for (auto & armor : find_armors)
        {
            if(armor.id == 1)
            {
                enemy_armor = armor;
                break;
            }
            armor.world_position = AS.pixel2imu(armor);
            double dis_tmp = armor.world_position.norm();
            if (dis_tmp < distance)
            {
                enemy_armor = armor;
                distance = dis_tmp;
            }
        }
        tracker_state = DETECTING;
        tracking_id = enemy_armor.id;
        

        // initial KF --- x_post
        KF.initial_KF();
        enemy_armor.world_position = AS.pixel2imu(enemy_armor);
        KF.setXPost(enemy_armor.world_position);
        Singer.setXpos({enemy_armor.world_position(0,0),enemy_armor.world_position(1,0)});
        // std::cout<<enemy_armor.camera_position.norm()<<"     "<<enemy_armor.world_position.norm()<<std::endl;
        // std::cout<<"enemy_armor.camera_position:  "<<enemy_armor.camera_position.transpose()<<std::endl;
        // std::cout<<AS.ab_roll<<"     "<<AS.ab_pitch<<"     "<<AS.ab_yaw<<std::endl;
        // std::cout<<"enemy_armor.world_position:  "<<enemy_armor.world_position.transpose()<<std::endl;

        return true;
    }

    bool ArmorTracker::switchEnemy(std::vector<Armor> find_armors)
    {
        Armor tmp;
        // genju juli shezhi fadanmoshi
        enemy_armor.world_position = AS.pixel2imu(enemy_armor);
//  std::cout<<"-------------------"<<find_armors.size()<<"   "<<enemy_armor.world_position.norm()<<"    "<<max_effective_distance<<std::endl;
        if(fabs(enemy_armor.world_position.norm() - max_effective_distance) < 1e-7)
        {
            reset();
            std::cout<<"over max_effective_distance"<<std::endl;
            return false;
        }
        else
        {
           // TODO: 如果另一台车比跟踪的进应该怎么选？
           if(find_armors.empty())
           {
                memset(switch_enemy_cnt, 0, sizeof switch_enemy_cnt);
                memset(flag, false, sizeof flag);
                memset(temp, 0, sizeof temp);
                return true;
           }
           else
           {
                memset(flag, false, sizeof flag);
                for(auto armor : find_armors)
                {
                    double tmp_dis = AS.pixel2imu(armor).norm() - enemy_armor.world_position.norm();
                    // std::cout<<"----------"<<tmp_dis<<"----------\n";
                    // std::cout<<"iiiiiiiiiiiiiiiiiiiiiidddddddddddddddddddddddd"<<armor.id<<std::endl;
                    bool a = tmp_dis >= delta_distance ? true:false;
                    // std::cout<<"aaaaaaaaaaaaaaaaaaa"<<a<<std::endl;
                    if(armor.id == 1)
                    {
                        flag[0] = true;
                        switch_enemy_cnt[0]++;
                        temp[0] = armor;
                    }
                    else if(/*(armor.id != 2) 
                    && */(fabs(tmp_dis - delta_distance) >= 1e-7)
                    /*&&  (armor.id != enemy_armor.id)*/)
                    {
                        // std::cout<<"NNNNNNNNNNNNNNNNNNNNNNNNNOOOOOOOOOOOOOOOOO\n";
                        switch(armor.id)
                        {
                            case 3:
                                flag[1] = true;
                                switch_enemy_cnt[1]++;
                                temp[1] = armor;
                                break;
                            case 4:
                                flag[2] = true;
                                switch_enemy_cnt[2]++;
                                temp[2] = armor;
                                break;
                            case 5:
                                flag[3] = true;
                                switch_enemy_cnt[3]++;
                                temp[3] = armor;
                                break;
                            case 6:
                                flag[4] = true;
                                switch_enemy_cnt[4]++;
                                temp[4] = armor;
                                break;
                        }
                    }
                }

                bool change = false;
                for(int i = 0; i < 5; i++)
                {
                    if(flag[i] == false)
                    {
                        switch_enemy_cnt[i] = 0;
                    }
                    // std::cout<<"44444444444444444444444    "<<switch_enemy_cnt[i]<<std::endl;
                    if(switch_enemy_cnt[i] >= switch_enemy_threshold)
                    {
                        // TODO: complex deal with this switch armor by using time
                        // if(!is_switch_time_set)
                        // {
                        //     last_change_time = std::chrono::high_resolution_clock::now();
                        //     is_switch_time_set = true;
                        // }
                        // chrono_time now = std::chrono::high_resolution_clock::now();
                        // double dt = seconds_duration(now - last_change_time).count();

                        // if(dt > switch_time_threshold)
                        // {
                            
                        // }
                        
                        
                        if(i == 0 && AS.pixel2imu(temp[i]).norm() <= hero_distance)
                        {//英雄
                            switch_enemy_cnt[i] = 0;
                            enemy_armor = temp[i];
                            change = true;
                            break;
                            
                        }
                        else if(AS.pixel2imu(temp[i]).norm() < AS.pixel2imu(enemy_armor).norm())
                        {
                            enemy_armor = temp[i];
                            change = true;
                        }
                    }
                }

                if(change)
                {
                    // std::cout<<'**********************************\n';
                    reset();
                    tracking_id = enemy_armor.id;
                    return false;
                }
                else
                {
                    return true;
                }
           }
       }
    }

    // dt是两帧之间时间间隔, 跟得住目标  用预测来做匹配，确定跟踪器状态
    bool ArmorTracker::selectEnemy(std::vector<Armor> find_armors, double dt)
    {
        KF.setF(dt);
        predicted_enemy = KF.predict();
        Armor matched_armor;
        bool matched = false;

    #ifdef ANTI_SPIN
        SpinHeading spin_status;
        if (!find_armors.empty()) {
            spin_status = spin_status_map[tracking_id];
            if (spin_status == UNKNOWN) {
    //            std::cout << "unkown" << std::endl;
                double min_position_diff = DBL_MAX;
                for(auto & armor : find_armors)
                {
                    armor.world_position = AS.pixel2imu(armor);
                    Eigen::Vector3d pre = predicted_enemy.head(3);
                    pre = enemy_armor.world_position;
                    double position_diff = (pre - armor.world_position).norm();

                    if (position_diff < min_position_diff)
                    {
                        min_position_diff = position_diff;
                        matched_armor = armor;
                    }
                }

                // std::cout<<"min_position_diff(m):    "<<min_position_diff<<std::endl;
                // std::cout<<"match_id: "<<matched_armor.id<<std::28endl;
                // 这个相同id对遮挡情况似乎不好
                if (min_position_diff < new_old_threshold && matched_armor.id == tracking_id)
                {
                    // std::cout<<"yes"<<std::endl;
                    matched = true;
                    predicted_enemy = KF.update(matched_armor.world_position);
                    switch_armor_cnt = 0;
                }
                else
                {
                    // std::cout<<"no"<<std::endl;
                    // std::cout<<"match_id: "<<matched_armor.id<<" || min_position_diff(m):    "<<min_position_diff<<std::endl;
                    // 本帧内是否有相同ID
                    // double same_armor_distance = DBL_MAX;
                    for (auto & armor : find_armors)
                    {
                        // double dis_tmp = (enemy_armor.world_position - armor.world_position).norm();
                        // std::cout<<tracking_id<<"    "<<armor.id<<"   "<<dis_tmp<<std::endl;
                        if (armor.id == tracking_id)
                        {
                            if (switch_armor_cnt <= switch_armor_threshold)
                            {
                                matched_armor = enemy_armor;
                                predicted_enemy = KF.update(matched_armor.world_position);
                                // std::cout<<"track_temp_lost!!   " << switch_armor_cnt <<std::endl;
                                switch_armor_cnt ++;
                            }
                            else
                            {
                                KF.initial_KF();
                                Eigen::VectorXd position_speed(6);
                                position_speed << armor.world_position, predicted_enemy.tail(3);
                                KF.setPosAndSpeed(armor.world_position, predicted_enemy.tail(3));
                                predicted_enemy = position_speed;

                                Singer.setXpos({armor.world_position[0],armor.world_position[1]});


                                matched_armor = armor;
                                switch_armor_cnt = 0;
                                std::cout<<"track_sameID_initial!!"<<std::endl;
                                // std::cout<<"same_armor_distance(m):    "<<same_armor_distance<<std::endl;
                            }
                            matched = true;
                            break;
                        }
                    }
                }
                is_vir_armor = false;
            }
            else  // spin_status != UNKNOWN
            {
                jump_tracker = {};
                auto ID_candiadates = trackers_map.equal_range(tracking_id);
                auto backward_with_same_ID = trackers_map.count(tracking_id);  // 已存在类型的预测器数量

                std::vector<Armor> final_armors;
                for (auto &armor: find_armors) {
                    if (armor.id == tracking_id)  // 同一帧图像的装甲板
                    {
                        armor.world_position = AS.pixel2imu(armor);
                        final_armors.push_back(armor);
                        // 储存装甲板
                        update_history_armors(armor);
                        matched = true;
                    } else {
                        continue;
                    }
                }
                //若存在一块装甲板
                if (final_armors.size() == 1)
                {
                    int kf_state = 0; // 1: 1->1(跳变); 2: 1->1/2->1(无跳变); 3: 虚拟装甲板
                    matched_armor = final_armors.at(0);
                    if (last_final_armors_size == 1 &&  // 单装甲板跳变:同时发生装甲板出现和消失
                        backward_with_same_ID == 1 &&
                        (matched_armor.world_position - trackers_map.find(tracking_id)->second.last_armor.world_position).norm()>new_old_threshold)
                    {
                        printf("---1->1(跳变)---\n");
                        update_spin_T(matched_armor, final_armors);
                        update_jump_trackers(matched_armor);
                        save_spin_history();
                        //disappear_tracker.disappear_armor = jump_trackers.at(0).jump_armor;
                        jump_trackers.erase(jump_trackers.begin());

                        if(is_vir_armor)kf_state = 2;
                        else kf_state = 1;
                    }
                    else  // 二/一 到一，追踪；无变化
                    {
                        if(last_final_armors_size == 2)
                        {
                            printf("---2->1(无跳变)---\n");
                            update_spin_T(matched_armor, final_armors);
                        }
                        kf_state = 2;
                    }
                    // 如果超过限制时间，构造虚拟装甲板。
                    double delay_time = AS.getFlyTime(matched_armor.world_position) * 1000 + 5;
                    printf("delay_time: {%lf}", delay_time);
    //                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch());
    //                auto ms2 = std::chrono::duration_cast<std::chrono::milliseconds>(jump_trackers.front().jump_time.time_since_epoch());
    //                std::cout << "t: " << ms.count() << std::endl;
    //                std::cout << "jump_time: " << ms2.count() << std::endl;
    //                std::cout << "jump_tracker_size: " << jump_trackers.size() << std::endl;
                    printf("duration:  {%lf}",  milliseconds_duration (t - jump_trackers.back().jump_time).count());
                    printf("T:  {%lf}", spin_T);

                    if(history_spin_state.size() > 1 &&
                    milliseconds_duration (t - jump_trackers.back().jump_time).count() >
                    std::get<0>(history_spin_state.front()) - delay_time)
                    {
                        printf("---generate virtual armor---\n");
                        real_armor = matched_armor;
                        /// 根据当前装甲板、前若干帧装甲板坐标用最小二乘法拟合圆，计算圆心、半径
                        std::vector<cv::Point2f> pts;
                        for (const auto& history_armor : history_armors)
                        {
                            pts.push_back(AS.Vector3d2point2f(history_armor.world_position));  // 将Eigen::Vector3d 转为 cv::point2f
                        }

                        pts.push_back(AS.Vector3d2point2f(jump_trackers.back().jump_armor.world_position));

                        Circle circle;
                        AS.circleLeastFit(pts, circle.x, circle.y, circle.r);
                        last_r = (circle.r + last_r) / 2;
                        printf("[r]: {%lf}", circle.r);
                        if (last_r > 0.35){
                            is_target_move = true;
                            spin_score_map[tracking_id] = 2000;  // 异常值退出反陀螺
                        }
                        else
                        {
                            double r = std::get<1>(history_spin_state.back());
                            double new_world_position_x;
                            double new_world_position_y;
                            // 该点与圆心连线和水平方向的夹角θ
                            double theta = atan2(matched_armor.world_position[1] - circle.y,
                                                matched_armor.world_position[0] - circle.x);
                            // 逆时针-，顺时针+
                            if (spin_status == COUNTER_CLOCKWISE) {
                                double alpha = theta - M_PI * 3 / 5;
                                new_world_position_x = r * cos(alpha) + circle.x;
                                new_world_position_y = r * sin(alpha) + circle.y;
                            } else
                            {
                                double beta = theta + M_PI * 3 / 5;
                                new_world_position_x = r * cos(beta) + circle.x;
                                new_world_position_y = r * sin(beta) + circle.y;
                            }
                            matched_armor.world_position.x() = new_world_position_x;
                            matched_armor.world_position.y() = new_world_position_y;
                            matched_armor.world_position.z() = std::get<2>(history_spin_state.back());
                        }
                        if (is_target_move)
                        {
                            kf_state = 1;  // TODO：验证是否可移除
                        }
                        else
                        {
                            kf_state = 3;
    //                        vir_num++;
                        }
                        if (vir_num > vir_max)
                        {
                            kf_state = 1;
                            spin_score_map[tracking_id] = 2000;
                        }
                    }
                    /// 初始化/更新KF参数
                    if(kf_state == 2)
                    {
                        predicted_enemy = KF.update(matched_armor.world_position);
                        is_vir_armor = false;
                    }
                    else if(kf_state == 3 && is_vir_armor)
                    {
                        predicted_enemy = KF.update(matched_armor.world_position);
                    }
                    else
                    {
                        reset_kf(matched_armor);
                        if(kf_state == 3 && !is_vir_armor)
                        {
                            is_aim_virtual_armor = true; // 出现虚拟装甲板后转过去
                            is_vir_armor = true;
                        }
                        if(kf_state == 1)is_vir_armor = false;
                    }
                    if (jump_trackers.size() >= 2)  // 维护jump_trackers
                    {
                        jump_trackers.erase(jump_trackers.begin());
                    }
    //                fmt::print("---generate over---");
                }
                //// 若存在两块装甲板
                else if (final_armors.size() == 2) {
                    // 对最终装甲板进行排序，选取与旋转方向相同的装甲板进行更新
                    sort(final_armors.begin(), final_armors.end(),
                        [](Armor &prev, Armor &next) { return prev.center.x < next.center.x; });
                    // 若顺时针旋转选取右侧装甲板
                    if (spin_status == CLOCKWISE)
                        matched_armor = final_armors.at(1);
                    // 若逆时针旋转选取左侧装甲板
                    else if (spin_status == COUNTER_CLOCKWISE)
                        matched_armor = final_armors.at(0);
                    matched = true;

                    // 一到二，跳变；新装甲板出现
                    if (last_final_armors_size == 1)
                    {
                        printf("---1->2(跳变)---\n");
                        update_jump_trackers(matched_armor);
                        save_spin_history();
                        if(is_vir_armor)
                        {
                            predicted_enemy = KF.update(matched_armor.world_position);
                        }
                        else
                        {
                            reset_kf(matched_armor);
                        }
                    }
                    else  // 二到二，追踪；无变化
                    {
                        printf("---2->2(无跳变)---\n");
                        predicted_enemy = KF.update(matched_armor.world_position);
                    }
                    if (is_vir_armor)is_vir_armor = false;
                }
                last_final_armors_size = final_armors.size();
            }
        }

    #else  // NOT_ANTI_SPIN
        if(!find_armors.empty())
        {
            double min_position_diff = DBL_MAX;
            for(auto & armor : find_armors)
            {
                armor.world_position = AS.pixel2imu(armor);
                Eigen::Vector3d pre = predicted_enemy.head(3);
                double position_diff = (pre - armor.world_position).norm();

                if (position_diff < min_position_diff)
                {
                    min_position_diff = position_diff;
                    matched_armor = armor;
                }
            }

            // std::cout<<"min_position_diff(m):    "<<min_position_diff<<std::endl;
            // std::cout<<"match_id: "<<matched_armor.id<<std::28endl;
            // 这个相同id对遮挡情况似乎不好
            if (min_position_diff < new_old_threshold && matched_armor.id == tracking_id)
            {
                // std::cout<<"yes"<<std::endl;
                matched = true;
                predicted_enemy = KF.update(matched_armor.world_position);
                switch_armor_cnt = 0;
            }
            else
            {
                // std::cout<<"no"<<std::endl;
                // std::cout<<"match_id: "<<matched_armor.id<<" || min_position_diff(m):    "<<min_position_diff<<std::endl;
                // 本帧内是否有相同ID
                // double same_armor_distance = DBL_MAX;
                for (auto & armor : find_armors)
                {
                    // double dis_tmp = (enemy_armor.world_position - armor.world_position).norm();
                    // std::cout<<tracking_id<<"    "<<armor.id<<"   "<<dis_tmp<<std::endl;
                    if (armor.id == tracking_id)
                    {
                        if (switch_armor_cnt <= switch_armor_threshold)
                        {
                            matched_armor = enemy_armor;
                            predicted_enemy = KF.update(matched_armor.world_position);
                            // std::cout<<"track_temp_lost!!   " << switch_armor_cnt <<std::endl;
                            switch_armor_cnt ++;
                        }
                        else
                        {
                            KF.initial_KF();
                            Eigen::VectorXd position_speed(6);
                            position_speed << armor.world_position, predicted_enemy.tail(3);
                            KF.setPosAndSpeed(armor.world_position, predicted_enemy.tail(3));

                            Singer.setXpos({armor.world_position[0],armor.world_position[1]});

                            predicted_enemy = position_speed;
                            matched_armor = armor;
                            switch_armor_cnt = 0;
                            std::cout<<"track_sameID_initial!!"<<std::endl;
                            // std::cout<<"same_armor_distance(m):    "<<same_armor_distance<<std::endl;
                        }
                        matched = true;
                        break;
                    }
                }
            }


        }
        else
        {
            // std::cout<<"input zero!!"<<std::endl;
        }

    #endif // ANTI_SPIN
        if (matched)
        {
            enemy_armor = matched_armor;
        }
        // predicted_position = predicted_enemy.head(3);
        // predicted_speed = predicted_enemy.tail(3);

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
    #ifdef ANTI_SPIN
                    history_armors.clear();
    #endif
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

        KF.setPosAndSpeed(enemy_armor.world_position,predicted_enemy.tail(3));
        if(tracker_state == LOSING)
        {
            enemy_armor.world_position = predicted_enemy.head(3);
            KF.setPosAndSpeed(enemy_armor.world_position,predicted_enemy.tail(3));
            if (spin_status != UNKNOWN)
            {

            }
        }
        return true;
    }

    // 对处于跟踪和正在丢失状态时做 预测，引入各种时间
    bool ArmorTracker::estimateEnemy(double dt)
    {
#ifdef ANTI_SPIN
        // 如果有虚拟装甲板的那一刻就重置Singer，然后把虚拟点接入Singer重新开始预测
        // if(is_aim_virtual_armor)
        // {
        //     // Singer.setXpos({enemy_armor.world_position[0],enemy_armor.world_position[1]});
        //     is_aim_virtual_armor = false;
        // }
#endif // ANTI_SPIN
        // if(is_aim_virtual_armor)
            KF.setXPost(enemy_armor.world_position);
            // KF.setPosAndSpeed(enemy_armor.world_position,predicted_enemy.tail(3));
        // std::cout<<"predict speed:   "<<predicted_enemy.tail(3).transpose()<<std::endl;

        KF.setF(
            dt
            + 1 
            + AS.getFlyTime(enemy_armor.world_position)
        );

        predicted_position = KF.predict().head(3);
        return true;
        
#ifdef PRE
        // 对跟踪状态和正在丢失状态时做预测
        if(tracker_state == TRACKING || tracker_state == LOSING)
        {
            double fly_time = AS.getFlyTime(enemy_armor.world_position);
            ////////////////Singer predictor//////////////////////////////
            if(!Singer.SingerPrediction(dt,fly_time,
                                        enemy_armor.world_position,
                                        predicted_position))
            {
                Singer.Reset({enemy_armor.world_position(0,0),enemy_armor.world_position(1,0)});
    //            Singer.setXpos({enemy_armor.world_position(0,0),enemy_armor.world_position(1,0)});
                std::cerr<<"[predict value illegal!!! Fix in origin value]"<<std::endl;
                return false;
            }
            ////////////////Singer predictor//////////////////////////////
            return true;
        }
        else if (tracker_state == DETECTING)
        {
            double fly_time = AS.getFlyTime(enemy_armor.world_position);
            ////////////////Singer predictor//////////////////////////////
            if(!Singer.SingerPrediction(dt,fly_time,
                                        enemy_armor.world_position,
                                        predicted_position))
            {
                // Singer.Reset({enemy_armor.world_position(0,0),enemy_armor.world_position(1,0)});
                // Singer.setXpos({enemy_armor.world_position(0,0),enemy_armor.world_position(1,0)});
                // std::cerr<<"[predict value illegal!!! Fix in origin value]"<<std::endl;
                return false;
            }
            ////////////////Singer predictor//////////////////////////////

            // 检测状态还给false就会一直进入initial函数，历史代码是这么写的，没问题
            // locate_target = false;
            return false;
        }
        else
        {
            // reset();  // 前一个函数变成MISSING会之间返回false，这个函数里这个判断无效
            // locate_target = false;
            return false;
        }
#endif

    }

    // 返回false保持现状，返回true开始控制    ddt --- chrono
    bool ArmorTracker::locateEnemy(const cv::Mat src, std::vector<Armor> armors, const chrono_time time)
    {
        src.copyTo(_src);
        // for infantry
        // AS.RotationMatrix_imu = AS.quaternionToRotationMatrix();
        
        if(!locate_target)
        {
            if(initial(armors))
            {
                locate_target = true;
            }
            else
            {
                locate_target = false;
            }
            return false;
        }
        else
        {
            if (wait_start)
            {
                t = time;
                wait_start = false;
                return false;
            }
            double dt = seconds_duration (time - t).count();
            t = time;

            
            if(!selectEnemy(armors,dt))  { return false; }

#ifdef ANTI_SPIN
            if (tracker_state == TRACKING) { spin_detect(); }
#endif

            // if(!switchEnemy(armors)) { return false; }

            if(!estimateEnemy(dt)) { return false;}

            //
            // Eigen::Vector3d gy;
            // if(is_aim_virtual_armor)
            // {
            //     Eigen::Vector3d rpy = AS.getAngle(enemy_armor.world_position,gy);
            //     pitch = rpy[1];
            //     yaw   = rpy[2];
            //     pitch = round(pitch * 100)/100;
            //     // std::cout<<"4"<<std::endl;
            //     return false;  // TODO: fire or fail
            // }
            // else
            // {
            //     Eigen::Vector3d rpy = AS.getAngle(predicted_position,gy);
            //     pitch = rpy[1];
            //     yaw   = rpy[2];
            //     pitch = round(pitch * 100)/100;
            // }
            
            // if(enemy_armor.world_position.norm() > 4)
            // {
            //     Eigen::Vector3d rpy = AS.getAngle(enemy_armor.world_position);
            //     pitch = rpy[1];if(switchEnemy(armors)) { return false; }

            //     yaw   = rpy[2];
            // }
            // else
            // {
            //     Eigen::Vector3d rpy = AS.getAngle(predicted_position);
            //     pitch = rpy[1];
            //     yaw   = rpy[2];
            //     pitch = round(pitch * 100)/100;
            // }

            
            Eigen::Vector3d rpy = AS.getAngle(enemy_armor.world_position,bullet_point);
            roll  = rpy[0];
            pitch = rpy[1];
            yaw   = rpy[2];
            pitch = round(pitch * 100)/100;

            // pitch = 0;

            if (pitch > 15)
            {
                reset();
                return false;
            }

            return true;
        }

    }

    /**
     * @brief 维护反陀螺状态map
     * @return 自然衰减状态分数，将分数低于阈值的去除，高于最大阈值的限制在最大值
     */
    bool ArmorTracker::updateSpinScore()
    {
        for (auto score = spin_score_map.begin(); score != spin_score_map.end();)
        {
            SpinHeading spin_status;
            //        std::cout << "score1: " << (*score).second << std::endl;

            //若Status_Map不存在该元素
            if (spin_status_map.count((*score).first) == 0)
                spin_status = UNKNOWN;
            else
                spin_status = spin_status_map[(*score).first];
            // 若分数过低移除此元素
            if (abs((*score).second) <= anti_spin_judge_low_thres && spin_status != UNKNOWN)
            {
                spin_status_map.erase((*score).first);
                score = spin_score_map.erase(score);
                is_anti = false;
                continue;
            }

            if (spin_status != UNKNOWN)
                (*score).second = 0.969 * (*score).second;
            else
                (*score).second = 0.987 * (*score).second;
            // 当小于该值时移除该元素
            if (abs((*score).second) < 40 || isnan((*score).second))
            {
                spin_status_map.erase((*score).first);
                score = spin_score_map.erase(score);
                continue;
            }
            else if (abs((*score).second) >= anti_spin_judge_high_thres)
            {
                (*score).second = anti_spin_judge_high_thres * abs((*score).second) / (*score).second;
                if ((*score).second > 0)
                    spin_status_map[(*score).first] = CLOCKWISE;
                else if((*score).second < 0)
                    spin_status_map[(*score).first] = COUNTER_CLOCKWISE;
                if(!is_anti)
                {
                    spin_T = 0;
                    last_final_armors_size = 0;
                    jump_trackers.clear();
                    jump_tracker.jump_armor = enemy_armor;
                    jump_tracker.jump_time = t;
                    jump_trackers.push_back(jump_tracker);
                    is_anti = true;
                    //std::cout << "start anti "<< std::endl;
                }
            }
            //        std::cout << "score2: " << (*score).second << std::endl;

            ++score;
        }

        // cout<<"++++++++++++++++++++++++++"<<endl;
        // for (auto status : spin_status_map)
        // {
        //     cout<<status.first<<" : "<<status.second<<endl;
        // }
        //    std::cout << "score over" << std::endl;

        return true;
    }

    /**
     * @brief 反陀螺状态检测
     */
    void ArmorTracker::spin_detect() {
        // std::cout << "---spin detect--- " << std::endl;

        Armor detect_armor;
        if(!is_vir_armor)
        {
            detect_armor = enemy_armor;

            //        std::cout << "---enemy_armor--- " << std::endl;
        }
        else
        {
            detect_armor = real_armor;
            //        std::cout << "---real_armor--- " << std::endl;
        }
        int armor_id = detect_armor.id;
        new_armors_cnt_map.clear();
        auto predictors_with_same_key = trackers_map.count(armor_id);  // 已存在类型的预测器数量
        if (predictors_with_same_key == 1)
        {
            auto candidate = trackers_map.find(armor_id);  // 原有的同ID装甲板
            auto delta_dist = (detect_armor.world_position - (*candidate).second.last_armor.world_position).norm();  // 距离
            //        std::cout << "[delta_dist1]: " << delta_dist << std::endl;
            if (delta_dist < new_old_threshold) {
                (*candidate).second.update_tracker(detect_armor, t);  // 更新装甲板
            }
            else {
                SpinTracker spinTracker(detect_armor, t); // 同类型不同位置创建
                trackers_map.insert(std::make_pair(armor_id, spinTracker));
                new_armors_cnt_map[armor_id]++;
                //    std::cout << "-----new_map-----" << std::endl;
                //    for (auto & it : trackers_map)
                //    {
                //        std::cout << it.first << " -> " << it.second.is_initialized << "\n";
                //    }
            }
        }
        else
        {
            // 1e9无实际意义，仅用于非零初始化
            double min_delta_dist = 1e9;
            double min_delta_t = 1e9;

            bool is_best_candidate_exist = false;
            std::multimap<int, SpinTracker>::iterator best_candidate;
            auto candiadates = trackers_map.equal_range(armor_id);
            for (auto iter = candiadates.first; iter != candiadates.second; ++iter) {
                auto delta_t = milliseconds_duration (t - (*iter).second.last_timestamp).count();
                auto delta_dist = (detect_armor.world_position - (*iter).second.last_armor.world_position).norm();

                // 在同一位置存在过装甲板且时间最接近设为最高优先级，
                if (delta_dist <= new_old_threshold && delta_dist <= min_delta_dist && delta_t < min_delta_t)  // 距离需要调试
                {
                    min_delta_dist = delta_dist;
                    min_delta_t = delta_t;
                    best_candidate = iter;
                    is_best_candidate_exist = true;
                    //                std::cout << "2->1" << std::endl;
                }
            }
            if (is_best_candidate_exist)
            {
                (*best_candidate).second.update_tracker(detect_armor, t);
            }
            else
            {   // first time or 2->1
                SpinTracker spinTracker(detect_armor, t);
                trackers_map.insert(std::make_pair(static_cast<int&&>(armor_id), static_cast<SpinTracker&&>(spinTracker)));
            }
        }

        //    std::cout << "[MAP_size]: "<< trackers_map.size() << std::endl;
        //
        if (!trackers_map.empty()) {
            //维护预测器Map，删除过久之前的装甲板
            for (auto iter = trackers_map.begin(); iter != trackers_map.end();) {
                //删除元素后迭代器会失效，需先行获取下一元素
                auto next = iter;
                if (milliseconds_duration(t - (*iter).second.last_timestamp).count() > max_delta_t)  //TODO：时间需要测试
                {
                    //                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch());
                    //                std::cout<<"timestamp : "<< ms.count() << std::endl;
                    //                auto ms2 = std::chrono::duration_cast<std::chrono::milliseconds>((*iter).second.last_timestamp.time_since_epoch());
                    //                std::cout<<"last_timestamp : "<< ms2.count() << std::endl;
                    //                std::cout<<"duration_cast : "<< milliseconds_duration (t - (*iter).second.last_timestamp).count() << std::endl;
                    //                std::cout << "[MAP_size]: "<< trackers_map.size() << std::endl;
                    next = trackers_map.erase(iter);
                }
                else
                    ++next;
                iter = next;
            }
        }
        //    std::cout << "[MAP_size_2]: "<< trackers_map.size() << std::endl;

        if (new_armors_cnt_map[armor_id] == 1) {
            //        std::cout << "-----spin detect-----" << std::endl;
            auto same_armors_cnt = trackers_map.count(armor_id);  // 相同的装甲板数量
            //        std::cout << "[same_armors_cnt] : "<< same_armors_cnt << std::endl;

            if (same_armors_cnt == 2) {
                //遍历所有同Key预测器，确定左右侧的Tracker
                SpinTracker *new_tracker = nullptr;
                SpinTracker *last_tracker = nullptr;
                double last_armor_center;
                //            chrono_time last_armor_timestamp;
                double new_armor_center;
                //            chrono_time new_armor_tisubscribermestamp;
                //            int best_prev_timestamp = 0;    //候选ArmorTracker的最近时间戳
                auto candiadates = trackers_map.equal_range(armor_id);  // 获取同类型的装甲板
                for (auto iter = candiadates.first; iter != candiadates.second; ++iter) {
                    //若未完成初始化则视为新增tracker
                    if (!(*iter).second.is_initialized) {
                        new_tracker = &(*iter).second;
                    } else {
                        last_tracker = &(*iter).second;
                    }
                }
                if (new_tracker != nullptr && last_tracker != nullptr) {

                    new_armor_center = new_tracker->last_armor.center.x;
                    //                new_armor_timestamp = new_tracker->last_timestamp;
                    last_armor_center = last_tracker->last_armor.center.x;
                    //                last_armor_timestamp = last_tracker->last_timestamp;
                    auto spin_movement = new_armor_center - last_armor_center;  // 中心x坐标： 新 - 旧
                    //                std::cout << "[spin_movement]: " << spin_movement << std::endl;

                    if (abs(spin_movement) > 10) {


                        // 若无该元素则插入新元素
                        if (spin_score_map.count(armor_id) == 0) {
                            spin_score_map[armor_id] = 1000 * spin_movement / abs(spin_movement);
                            //                        std::cout << "create new score" << std::endl;
                        }
                            //  若已有该元素且目前旋转方向与记录不同,则对目前分数进行减半惩罚
                        else if (spin_movement * spin_score_map[armor_id] < 0) {
                            spin_score_map[armor_id] *= 0.3;
                            //                        std::cout << "-----cut score-----" << std::endl;

                        }
                            // 若已有该元素则更新元素
                        else {
                            //                        std::cout << "update score" << std::endl;
                            spin_score_map[armor_id] = anti_spin_max_r_multiple * spin_score_map[armor_id];
                        }
                        for (auto & it : spin_score_map)
                        {
                            std::cout << it.first << " -> " << it.second << "\n";
                        }
                    }
                }
            }
        }
        for (auto & it : spin_score_map)
        {
            std::cout << it.first << " -> " << it.second << "\n";
        }
        updateSpinScore();

    }

    void ArmorTracker::update_history_armors(Armor new_armor) {
        if (history_armors.size() <= max_history_len)
        {
            history_armors.push_back(new_armor);
        } else
        {
            history_armors.pop_front();
            history_armors.push_back(new_armor);
        }
    }

    void ArmorTracker::update_jump_trackers(Armor new_armor) {
        jump_tracker.jump_armor = new_armor;
        jump_tracker.jump_time = t;
        jump_trackers.push_back(jump_tracker);
    }

    void ArmorTracker::reset_kf(Armor new_armor) {
        KF.initial_KF();
        Eigen::Matrix<double, 6, 1> pos_speed;
        pos_speed << new_armor.world_position, predicted_enemy.tail(3);
        KF.setPosAndSpeed(new_armor.world_position, predicted_enemy.tail(3));
        predicted_enemy = pos_speed;

    }

    void ArmorTracker::update_spin_T(Armor matched_armor,  std::vector<Armor> &final_armors) {
        if (milliseconds_duration (t - jump_trackers.at(0).jump_time).count() / spin_T > 0.3){
            if (spin_T == 0)
                spin_T = milliseconds_duration (t - jump_trackers.at(0).jump_time).count();
            else
                spin_T = milliseconds_duration (t - jump_trackers.at(0).jump_time).count() * 0.3 + spin_T * 0.7;
        }
        else
        {
            enemy_armor.world_position = predicted_enemy.head(3);
            matched_armor = enemy_armor;
            final_armors.push_back(matched_armor);
        }
    }

    void ArmorTracker::save_spin_history() {


        if (history_spin_state.size() < 2)
        {
            history_spin_state.emplace_back(spin_T, last_r, enemy_armor.world_position.z());
        } else
        {
            history_spin_state.pop_front();
            history_spin_state.emplace_back(spin_T, last_r, enemy_armor.world_position.z());
        }
    }


}