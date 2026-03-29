#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <astra_custom_msgs/MarkerMeasurement.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>



class EKFPredictor {
private:
    // EKF状态变量
    Eigen::VectorXd x_;          // 状态向量 [x, y, z, φ, v, a, K, curvature_speed, vz]
    Eigen::MatrixXd P_;          // 协方差矩阵
    Eigen::MatrixXd Q_;          // 过程噪声协方差
    Eigen::MatrixXd R_;          // 观测噪声协方差
    
    // ROS组件
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher vis_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // 参数
    double dt_;                  // 时间步长
    double predicted_time_;      // 预测时间
    bool first_measurement_;     // 首次测量标志
    
public:
    EKFPredictor() : 
        nh_("~"),
        tf_listener_(tf_buffer_),
        dt_(0.1),
        predicted_time_(1.2),
        first_measurement_(true)
    {
        // 初始化EKF状态（9维）
        x_ = Eigen::VectorXd::Zero(9);
        
        // 初始化协方差矩阵
        P_ = Eigen::MatrixXd::Identity(9, 9) * 10.0;
        
        // 初始化过程噪声协方差
        Q_ = Eigen::MatrixXd::Identity(9, 9) * 0.1;
        
        // 初始化观测噪声协方差（4维观测：x, y, z, φ）
        R_ = Eigen::MatrixXd::Zero(4, 4);
        R_(0, 0) = 0.1;  // x
        R_(1, 1) = 0.1;  // y
        R_(2, 2) = 0.1;  // z
        R_(3, 3) = 0.1;  // φ
        
        // 获取参数
        nh_.param("predicted_time", predicted_time_, 1.2);
        
        // 创建发布器
        marker_pub_ = nh_.advertise<astra_custom_msgs::MarkerMeasurement>(
            "/EKF/MarkerMeasurement/estimate", 10);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/EKF/PoseStamped/estimate", 10);
        vis_pub_ = nh_.advertise<visualization_msgs::Marker>(
            "/EKF/Marker/prediction", 10);
        
        ROS_INFO("EKF Car Model Node Started");
        ROS_INFO("Predicted time: %.2f seconds", predicted_time_);
    }
    
    // 非线性状态转移函数
    Eigen::VectorXd fx(const Eigen::VectorXd& x, double dt) {
        Eigen::VectorXd x_new = x;
        
        double x_pos = x(0);
        double y_pos = x(1);
        double z_pos = x(2);
        double phi = x(3);
        double v = x(4);
        double a = x(5);
        double K = x(6);
        double curvature_speed = x(7);
        double vz = x(8);
        
        // 非线性状态更新
        x_new(0) = x_pos + v * cos(phi) * dt;      // X位置更新
        x_new(1) = y_pos + v * sin(phi) * dt;      // Y位置更新
        x_new(2) = z_pos + vz * dt;                // Z位置更新
        x_new(3) = phi + K * v * dt;               // 偏航角更新
        x_new(4) = v + a * dt;                     // 速度更新
        x_new(6) = K + curvature_speed * dt;       // 曲率更新
        // vz保持不变（恒速模型）
        
        return x_new;
    }
    
    // 计算状态转移函数的雅可比矩阵
    Eigen::MatrixXd Fjacobian(const Eigen::VectorXd& x, double dt) {
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
        
        double phi = x(3);
        double v = x(4);
        double K = x(6);
        
        // XY平面模型部分的偏导数
        F(0, 4) = cos(phi) * dt;           // dx/dv
        F(0, 3) = -v * sin(phi) * dt;      // dx/dφ
        F(1, 4) = sin(phi) * dt;           // dy/dv
        F(1, 3) = v * cos(phi) * dt;       // dy/dφ
        F(3, 4) = K * dt;                  // dφ/dv
        F(3, 6) = v * dt;                  // dφ/dK
        
        // Z轴恒速模型部分
        F(2, 8) = dt;  // dz/dvz
        
        return F;
    }
    
    // 观测函数
    Eigen::VectorXd hx(const Eigen::VectorXd& x) {
        Eigen::VectorXd z(4);
        z << x(0), x(1), x(2), x(3);  // [x, y, z, φ]
        return z;
    }
    
    // 计算观测函数的雅可比矩阵
    Eigen::MatrixXd Hjacobian(const Eigen::VectorXd& x) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 9);
        H(0, 0) = 1;  // dx/dx
        H(1, 1) = 1;  // dy/dy
        H(2, 2) = 1;  // dz/dz
        H(3, 3) = 1;  // dφ/dφ
        return H;
    }
    
    // 预测步骤
    void predict() {
        // EKF预测步骤
        Eigen::MatrixXd F = Fjacobian(x_, dt_);
        x_ = fx(x_, dt_);
        P_ = F * P_ * F.transpose() + Q_;
        
        ROS_DEBUG_STREAM("Predicted state: x=" << x_(0) << ", y=" << x_(1) 
                        << ", z=" << x_(2) << ", phi=" << x_(3));
    }
    
    // 更新步骤
    void update(const Eigen::VectorXd& z) {
        // EKF更新步骤
        Eigen::VectorXd y = z - hx(x_);           // 观测残差
        Eigen::MatrixXd H = Hjacobian(x_);        // 计算观测矩阵
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_;  // 计算观测协方差
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();  // 计算卡尔曼增益
        
        // 更新状态和协方差
        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(9, 9) - K * H) * P_;
        
        ROS_DEBUG_STREAM("Updated state: x=" << x_(0) << ", y=" << x_(1) 
                        << ", z=" << x_(2) << ", phi=" << x_(3));
    }
    
    // 处理TF变换
    void processTF() {
        try {
            // 获取TF变换
            geometry_msgs::TransformStamped transform = 
                tf_buffer_.lookupTransform("map", "target", ros::Time(0), ros::Duration(1.0));
            
            // 创建观测向量
            Eigen::VectorXd z(4);
            z(0) = transform.transform.translation.x;
            z(1) = transform.transform.translation.y;
            z(2) = transform.transform.translation.z;
            
            // 获取yaw角（φ）
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            z(3) = yaw;
            
            // 如果是第一次测量，初始化状态
            if (first_measurement_) {
                ROS_INFO("First measurement received, initializing state...");
                x_(0) = z(0);  // 设置x位置
                x_(1) = z(1);  // 设置y位置
                x_(2) = z(2);  // 设置z位置
                x_(3) = z(3);  // 设置初始φ值
                // 其他状态保持为0
                first_measurement_ = false;
                ROS_INFO_STREAM("Initial state set: x=" << x_(0) << ", y=" << x_(1) 
                                << ", z=" << x_(2) << ", phi=" << x_(3));
            } else {
                // 执行EKF更新步骤
                update(z);
            }
            
            // 发布结果
            publishResults();
            
        } catch (tf2::TransformException &ex) {
            ROS_WARN("TF exception: %s", ex.what());
        }
    }
    
    // 发布所有结果
    void publishResults() {
        // 计算预测位置（在predicted_time时间后的位置）
        double predicted_x = x_(0) + x_(4) * cos(x_(3)) * predicted_time_;
        double predicted_y = x_(1) + x_(4) * sin(x_(3)) * predicted_time_;
        double predicted_z = x_(2) + x_(8) * predicted_time_;
        
        // 发布自定义消息
        astra_custom_msgs::MarkerMeasurement marker_msg;
        marker_msg.position.x = predicted_x;
        marker_msg.position.y = predicted_y;
        marker_msg.position.z = predicted_z;
        
        // 转换为四元数
        tf2::Quaternion q;
        q.setRPY(0, 0, x_(3));
        marker_msg.orientation.x = q.x();
        marker_msg.orientation.y = q.y();
        marker_msg.orientation.z = q.z();
        marker_msg.orientation.w = q.w();
        
        // 设置其他消息字段
        marker_msg.yaw = x_(3) * 180.0 / M_PI;  // 转换为度
        marker_msg.v = x_(4);
        marker_msg.a = x_(5);
        marker_msg.K = x_(6);
        marker_msg.K_speed = x_(7);
        marker_msg.vz = x_(8);
        
        marker_pub_.publish(marker_msg);
        
        // 发布当前位置作为PoseStamped
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = x_(0);
        pose_msg.pose.position.y = x_(1);
        pose_msg.pose.position.z = x_(2);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        
        pose_pub_.publish(pose_msg);
        
        // 可视化预测点
        visualization_msgs::Marker vis_msg;
        vis_msg.header.frame_id = "map";
        vis_msg.header.stamp = ros::Time::now();
        vis_msg.ns = "land_point";
        vis_msg.id = 0;
        vis_msg.type = visualization_msgs::Marker::SPHERE;
        vis_msg.action = visualization_msgs::Marker::ADD;
        
        vis_msg.pose.position.x = predicted_x;
        vis_msg.pose.position.y = predicted_y;
        vis_msg.pose.position.z = predicted_z;
        
        vis_msg.scale.x = 0.1;
        vis_msg.scale.y = 0.1;
        vis_msg.scale.z = 0.1;
        
        vis_msg.color.a = 1.0;
        vis_msg.color.r = 1.0;
        vis_msg.color.g = 0.0;
        vis_msg.color.b = 0.0;
        
        vis_pub_.publish(vis_msg);
    }
    
    void run() {
        // 创建定时器用于预测步骤
        ros::Timer timer = nh_.createTimer(
            ros::Duration(dt_), 
            [this](const ros::TimerEvent&) { this->predict(); });
        
        ROS_INFO("Starting main loop...");
        ros::Rate rate(1.0 / dt_);  // 控制循环频率
        
        while (ros::ok()) {
            processTF();
            ros::spinOnce();
            rate.sleep();
        }
        
        ROS_INFO("EKF prediction node shutting down...");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf_car_model");
    
    // 设置日志级别（可选）
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
        ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    try {
        EKFPredictor predictor;
        predictor.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    
    return 0;
}