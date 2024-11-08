#include <memory>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "rogidrive_msg/msg/rogidrive_message.hpp"
#include "Dif_Everything.hpp"

using std::placeholders::_1;
 
class Shoot : public rclcpp::Node
{
public:
    Shoot();
 
private:
    void topic_callback(const geometry_msgs::msg::Pose & msg);

    void getVelocity(float x_r_, float y_r_, float theta_r_, float theta_l_pre, bool high_or_low);
    float getYaw(float x_r_, float y_r_, float theta_r_);
    float getPitch_thetadomain(float x_r_, float y_r_, float theta_l_pre); //ボールの落着角が基準のピッチ角
    float getV_thetadomain(float x_r_, float y_r_, float theta_l_pre); //ボールの落着角が基準の初速
    float getPitch_maxV_high(float x_r_, float y_r_);    //最大速度で撃つときのピッチ角（高い方）
    float getPitch_maxV_low(float x_r_, float y_r_); //最大速度で撃つときのピッチ角（低い方）

    // void calc_pose(float x_r, float y_r, float theta_l);     //いつか機体の設計が固まったら射出口の自己位置を出すために使いたいものだなぁ（感嘆）

    bool high_or_low;   //pitchがhighかlowかの確認

    float theta_pitch;  //射出装置の仰角(rad)
    float theta_pitch_high; //射出装置の仰角(rad)候補＠１
    float theta_pitch_low;  //射出装置の仰角(rad)候補＠２

    float v_thetadomein;    //ボールの入射角を基準に決められる速さ
    float pitch_thetadomein;    //ボールの入射角を基準に決められるピッチ角

    float pitch_vdomein;    //最大射出速度で撃つ時のピッチ角

    float theta_l_pre;  //naosumaeno bo-ru no nyuusyakaku
    float theta_l;  //ボールの入射角

    float theta_g;  //ローカル座標のx軸とローカル座標中心とゴールを結んだ直線のなす角

    float x_s;        //射出装置のx座標
    float y_s;        //射出装置のy座標
    float theta_s;    //射出装置のヨー角

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_micon;
    rclcpp::Publisher<rogidrive_msg::msg::RogidriveMessage>::SharedPtr publisher_odrive;

    std::array<float, 3> currentPose;   //射出装置のx座標, 射出装置のy座標, 射出装置のヨー角
    std::array<float, 3> shootVelocity; //speed, pitch, yaw(射撃諸元)
};

Shoot::Shoot()
: Node("shoot")
{
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>("currentPose", 10, std::bind(&Shoot::topic_callback, this, _1));
    publisher_micon = this->create_publisher<std_msgs::msg::Float64MultiArray>("shootVelocity", 10);
    publisher_odrive = this->create_publisher<rogidrive_msg::msg::RogidriveMessage>("odrive_cmd", 10);

}

float Shoot::getYaw(float _x_s, float _y_s, float _theta_r){
    float theta_yaw = atan((y_g - _y_s) / (x_g - _x_s)) - _theta_r;

    return theta_yaw;
}

float Shoot::getV_thetadomain(float x_s_, float y_s_, float theta_l_pre){
    theta_l = 2 * M_PI - theta_l_pre;

    float l = hypot(y_g - y_s_, x_g - x_s_);

    pitch_thetadomein = atan(((2*h)/l) - tan(theta_l));

    v_thetadomein = (l/cos(pitch_thetadomein)) * sqrt(l/(2*((l*tan(pitch_thetadomein)) - h)));
    return v_thetadomein;
}

float Shoot::getPitch_thetadomain(float x_s_, float y_s_, float theta_l_pre){
    theta_l = 2 * M_PI - theta_l_pre;

    float l = hypot(y_g - y_s_, x_g - x_s_);

    pitch_thetadomein = atan(((2*h)/l) - tan(theta_l));

    v_thetadomein = (l/cos(pitch_thetadomein)) * sqrt(l/(2*((l*tan(pitch_thetadomein)) - h)));
    return pitch_thetadomein;
}

float Shoot::getPitch_maxV_high(float x_s_, float y_s_){
    float l = hypot(y_g - y_s_, x_g - x_s_);

    if((1.0 - ((2.0*g / (v_max*v_max)) * (h + (l*l * g / (2.0*v_max*v_max))))) < 0)
    {
        return 0;
    }
    else
    {
        theta_pitch_high = atan((v_max*v_max) * (1.0 + sqrt(1 - ((2.0*g / (v_max*v_max)) * (h + (l*l * g / (2.0*v_max*v_max)))))) / (l * g));
    
        return theta_pitch_high;
    }
}

float Shoot::getPitch_maxV_low(float x_s_, float y_s_){
    float l = hypot(y_g - y_s_, x_g - x_s_);

    if((1.0 - ((2.0*g / (v_max*v_max)) * (h + (l*l * g / (2.0*v_max*v_max))))) < 0)
    {
        return 0;
    }
    else
    {
        theta_pitch_low = atan((v_max*v_max) * (1.0 - sqrt(1 - ((2.0*g / (v_max*v_max)) * (h + (l*l * g / (2.0*v_max*v_max)))))) / (l * g));
    
        return theta_pitch_low;
    }
}

// void Shoot::calc_pose(float x_r, float y_r, float theta_r){
//     x_s = x_r - l_1*sin(theta_r) - l_2*sin(theta_yaw);
//     y_s = y_r + l_1*cos(theta_r) - l_2*cos(theta_yaw);
// }    //いつか機体の設計が固まったら射出口の自己位置を出すために使いたいものだなぁ（感嘆）

void Shoot::getVelocity(float _x_r, float _y_r, float _theta_r, float _theta_l_pre, bool high_or_low){

    // calc_pose(x_r, y_r, theta_r);

    shootVelocity[2] = getYaw(_x_r, _y_r, _theta_r);

    if(getV_thetadomain(_x_r, _y_r, _theta_l_pre) <= v_max)
    {
        shootVelocity[1] = getPitch_thetadomain(_x_r, _y_r, _theta_l_pre);
        shootVelocity[0] = getV_thetadomain(_x_r, _y_r, _theta_l_pre);
    }
    else
    {
        shootVelocity[0] = v_max;
        if(high_or_low)
        {
            shootVelocity[1] = getPitch_maxV_high(_x_r, _y_r);
        }
        else
        {
            shootVelocity[1] = getPitch_maxV_low(_x_r, _y_r);
        }
    }
}

void Shoot::topic_callback(const geometry_msgs::msg::Pose & msg)
{
    auto message_micon = std_msgs::msg::Float64MultiArray();

    tf2::Quaternion q(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll;
    double pitch;
    double yaw;
    m.getRPY(roll, pitch, yaw);

    getVelocity(msg.position.x, msg.position.y, yaw, theta_l_pre, high_or_low);

    message_micon.data = {shootVelocity[0], shootVelocity[1], shootVelocity[2]};

    publisher_micon->publish(message_micon);

    auto message_odrive = rogidrive_msg::msg::RogidriveMessage();
    message_odrive.name = "motor1";
    message_odrive.mode = 0;
    message_odrive.vel = (shootVelocity[0] / radius) * k_rot;
    message_odrive.pos = 0;
    publisher_odrive->publish(message_odrive);

    // RCLCPP_INFO(this->get_logger(), "I heard position: [x: %f, y: %f, z: %f]", msg.position.x, msg.position.y, msg.position.z);
    // RCLCPP_INFO(this->get_logger(), "I heard orientation: [yaw: %f]", yaw);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Shoot>());
  rclcpp::shutdown();
  return 0;
}