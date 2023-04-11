#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
// #include "robot_status.h"
#include "spin_track.h"
#include "armor_detection.h"
#include "armor_prediction.h"
#include "singer_prediction.h"
#include "gimbal_control.h"


// 目的：通过读取进来的armors，筛选出同ID和上一帧的的装甲板，做跟踪

namespace robot_detection {

    enum TrackerState {
        MISSING,     // 没有目标
        DETECTING,   // 还未开始跟踪，作为跟踪第一帧的切换，初始化好预测
        LOSING,      // 处于丢失状态，还会保留预测
        TRACKING,    // 处于跟踪状态
    };

    struct Circle
    {
        double x; // 圆心x坐标
        double y; // 圆心y坐标
        double r; // 半径
    };
    struct Jump_tracker {
        Jump_tracker() = default;
        chrono_time jump_time;
        Armor jump_armor;
    };
    struct Disappear_tracker{
        Disappear_tracker() = default;
        chrono_time disappear_time;
        Armor disappear_armor;
    };

    class ArmorTracker
    {
    public:
        cv::Mat _src;
        float pitch;
        float yaw;
        chrono_time t;
        bool wait_start;

        ArmorTracker();

        void reset();

        bool initial(std::vector<Armor> find_armors);

        bool switchEnemy(std::vector<Armor> find_armors);

        bool selectEnemy(std::vector<Armor> find_armors, double dt);

        bool estimateEnemy(double dt);

        bool locateEnemy(cv::Mat src, std::vector<Armor> armors, chrono_time time);

        Circle fitCircle(const cv::Point2f& x1, const cv::Point2f& x2, const cv::Point2f& x3);
        bool updateSpinScore();
        void spin_detect();

        double last_r = 0.35;
        Eigen::Vector3d last_position;
        bool is_vir_armor = false;
        bool is_anti = false;
        int last_final_armors_size;
        double anti_spin_max_r_multiple;         // 符合陀螺条件，反陀螺分数增加倍数
        int anti_spin_judge_low_thres;           // 小于该阈值认为该车已关闭陀螺
        int anti_spin_judge_high_thres;          // 大于该阈值认为该车已开启陀螺
        int max_delta_t;                    //使用同一预测器的最大时间间隔(ms)
    //    double max_delta_dist;               // 最大追踪距离
        double spin_T;
        int vir_max = 20;
        int vir_num = 0;
        Jump_tracker jump_tracker;
        Disappear_tracker disappear_tracker;
        std::vector<Jump_tracker> jump_trackers;

        std::map<int,int> new_armors_cnt_map;          // 装甲板计数map，记录新增装甲板数
        std::multimap<int, SpinTracker> trackers_map;  // 预测器Map
        std::map<int,SpinHeading> spin_status_map;     // 记录该车小陀螺状态（未知，顺时针，逆时针）
        std::map<int,double> spin_score_map;           // 记录各装甲板小陀螺可能性分数，大于0为逆时针旋转，小于0为顺时针旋转
        std::deque<Armor> history_armors;
        const int max_history_len = 4;
        
        bool is_aim_virtual_armor;  // 出现虚拟装甲板后转过去

        Armor enemy_armor;//最终选择的装甲板
        Armor real_armor; // virtual armor state, real armor
        Eigen::Matrix<double,6,1> predicted_enemy;
        Eigen::Vector3d predicted_position;  // 预测的坐标，也是要发送给电控角度的坐标计算的角度
        Eigen::Vector3d predicted_speed;  // 预测得到的速度
        Eigen::Vector3d bullet_point;

        AngleSolve AS;
        // Q和R分别代表对预测值和测量值的置信度（反比），通过影响卡尔曼增益K的值，影响预测值和测量值的权重。越大的R代表越不相信测量值。
        KalmanFilter KF;
        Skalman Singer;

        bool locate_target;

        int tracker_state;  // 此时跟踪器的状态
        int tracking_id;    // 跟踪的敌方ID

        int find_aim_cnt;
        int find_threshold;

        int lost_aim_cnt;  // 丢失目标计数
        int lost_threshold;

        int prediction_false_cnt;  // 当预测的结果不正确时，连续计数超过阈值，重置

        double shoot_delay;

        double new_old_threshold; // 新旧坐标的距离阈值

        int switch_enemy_cnt;
        int switch_enemy_threshold;
        double max_effective_distance;
    };

}
