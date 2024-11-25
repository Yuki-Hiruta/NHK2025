#include <memory>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rogidrive_msg/msg/rogidrive_message.hpp"
#include "rogidrive_msg/msg/rogidrive_multi_array.hpp"
#include "Dif_Everything.hpp"

using std::placeholders::_1;

class Shoot : public rclcpp::Node
{
public:
    Shoot();

private:
    void topic_callback_pose(const geometry_msgs::msg::Pose2D & msg);
    void topic_callback_controller(const sensor_msgs::msg::Joy & msg);
    void topic_callback_odrive_pos(const rogidrive_msg::msg::RogidriveMultiArray & msg);

    float pid(float _current_pos, float target_pos, float kp, float ki, float kd);
    void pid_reset();

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

    float x_r;
    float y_r;
    float theta_r;

    float x_s;        //射出装置のx座標
    float y_s;        //射出装置のy座標
    float theta_s;    //射出装置のヨー角

    bool commence_fire;

    // float frequency is defined in Dif_Ev
    float error;
    float preverror;
    float integral;
    float output;

    float torque;
    float k_torque;
    float limit_pos;
    float torque_constant;

    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscription_pose;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_controller;
    rclcpp::Subscription<rogidrive_msg::msg::RogidriveMultiArray>::SharedPtr subscription_odrive_pos;

    rclcpp::Publisher<rogidrive_msg::msg::RogidriveMessage>::SharedPtr publisher_odrive;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_micon;

    std::array<float, 3> currentPose;   //射出装置のx座標, 射出装置のy座標, 射出装置のヨー角
    std::array<float, 3> shootVelocity; //speed, pitch, yaw(射撃諸元)
    std::vector<double> motor_pos;
};

Shoot::Shoot()
: Node("shoot")
, high_or_low(true)
, commence_fire(false)
, shootVelocity({0, 0, 0})
{
    this->declare_parameter("k_torque", 1.0);
    this->declare_parameter("limit_pos", 2.0);
    this->declare_parameter("torque_constant", 2.0);
    this->declare_parameter("theta_l_pre", M_PI/4);

    k_torque = (float)this->get_parameter("k_torque").as_double();
    limit_pos = (float)this->get_parameter("limit_pos").as_double();
    torque_constant = (float)this->get_parameter("torque_constant").as_double();
    theta_l_pre = (float)this->get_parameter("theta_l_pre").as_double();

    //自己位置を取得
    subscription_pose = this->create_subscription<geometry_msgs::msg::Pose2D>("currentPose", 10, std::bind(&Shoot::topic_callback_pose, this, _1));

    //joy_nodeからうけとり
    subscription_controller = this->create_subscription<sensor_msgs::msg::Joy>("joy_controller", 10, std::bind(&Shoot::topic_callback_controller, this, _1));

    //odriveのエンコーダを読み取り
    subscription_odrive_pos = this->create_subscription<rogidrive_msg::msg::RogidriveMultiArray>("rogidrive_status", 10, std::bind(&Shoot::topic_callback_odrive_pos, this, std::placeholders::_1));

    //マイコンに砲塔の諸元を送る
    publisher_micon = this->create_publisher<std_msgs::msg::Float64MultiArray>("shootVelocity", 10);

    //rogidriveにトルクを送る
    publisher_odrive = this->create_publisher<rogidrive_msg::msg::RogidriveMessage>("odrive_cmd", 10);

}

//pid制御
float Shoot::pid(float _current_pos, float _target_pos, float kp, float ki, float kd)
{
    error = _target_pos - _current_pos;
    output = kp * error + ki/shooter::frequency * integral + kd*shooter::frequency * (error - preverror);

    preverror = error;
    integral += error;

    return output;
}

//preverrorとintegralを0にリセット
void Shoot::pid_reset()
{
    integral = 0;
    preverror = 0;
}

//yaw角を算出
float Shoot::getYaw(float _x_s, float _y_s, float _theta_r){
    float theta_yaw = _theta_r - atan((shooter::x_g - _x_s) / (shooter::y_g - _y_s));

    return theta_yaw;
}

//落着角度から砲塔pitch角を算出
float Shoot::getV_thetadomain(float x_s_, float y_s_, float theta_l_pre){
    theta_l = 2 * M_PI - theta_l_pre;

    float l = hypot(shooter::y_g - y_s_, shooter::x_g - x_s_);

    pitch_thetadomein = atan(((2*shooter::h)/l) - tan(theta_l));

    v_thetadomein = (l/cos(pitch_thetadomein)) * sqrt(l/(2*((l*tan(pitch_thetadomein)) - shooter::h)));
    return v_thetadomein;
}

//落着角度から砲塔射撃速度を算出
float Shoot::getPitch_thetadomain(float x_s_, float y_s_, float theta_l_pre){
    theta_l = 2 * M_PI - theta_l_pre;

    float l = hypot(shooter::y_g - y_s_, shooter::x_g - x_s_);

    pitch_thetadomein = atan(((2*shooter::h)/l) - tan(theta_l));

    v_thetadomein = (l/cos(pitch_thetadomein)) * sqrt(l/(2*((l*tan(pitch_thetadomein)) - shooter::h)));
    return pitch_thetadomein;
}

//最大射撃速度から砲塔pitch角を算出
float Shoot::getPitch_maxV_high(float x_s_, float y_s_){
    float l = hypot(shooter::y_g - y_s_, shooter::x_g - x_s_);

    if((1.0 - ((2.0*shooter::g / (shooter::v_max*shooter::v_max)) * (shooter::h + (l*l * shooter::g / (2.0*shooter::v_max*shooter::v_max))))) < 0)
    {
        return 0;
    }
    else
    {
        theta_pitch_high = atan((shooter::v_max*shooter::v_max) * (1.0 + sqrt(1 - ((2.0*shooter::g / (shooter::v_max*shooter::v_max)) * (shooter::h + (l*l * shooter::g / (2.0*shooter::v_max*shooter::v_max)))))) / (l * shooter::g));
    
        return theta_pitch_high;
    }
}

//最大射撃速度から砲塔射撃速度を算出
float Shoot::getPitch_maxV_low(float x_s_, float y_s_){
    float l = hypot(shooter::y_g - y_s_, shooter::x_g - x_s_);

    if((1.0 - ((2.0*shooter::g / (shooter::v_max*shooter::v_max)) * (shooter::h + (l*l * shooter::g / (2.0*shooter::v_max*shooter::v_max))))) < 0)
    {
        return 0;
    }
    else
    {
        theta_pitch_low = atan((shooter::v_max*shooter::v_max) * (1.0 - sqrt(1 - ((2.0*shooter::g / (shooter::v_max*shooter::v_max)) * (shooter::h + (l*l * shooter::g / (2.0*shooter::v_max*shooter::v_max)))))) / (l * shooter::g));
    
        return theta_pitch_low;
    }
}

//いつか機体の設計が固まったら射出口の自己位置を出すために使いたいものだなぁ（感嘆）
// void Shoot::calc_pose(float x_r, float y_r, float theta_r){
//     x_s = x_r - l_1*sin(theta_r) - l_2*sin(theta_yaw);
//     y_s = y_r + l_1*cos(theta_r) - l_2*cos(theta_yaw);
// }

//射撃諸元を取得
void Shoot::getVelocity(float _x_r, float _y_r, float _theta_r, float _theta_l_pre, bool high_or_low){

    // calc_pose(x_r, y_r, theta_r);

    shootVelocity[2] = getYaw(_x_r, _y_r, _theta_r);

    if(getV_thetadomain(_x_r, _y_r, _theta_l_pre) <= shooter::v_max)
    {
        shootVelocity[1] = getPitch_thetadomain(_x_r, _y_r, _theta_l_pre);
        shootVelocity[0] = getV_thetadomain(_x_r, _y_r, _theta_l_pre);
    }
    else
    {
        shootVelocity[0] = shooter::v_max;
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

//自己位置を定期的にsubし、そのたびに射撃諸元を算出
void Shoot::topic_callback_pose(const geometry_msgs::msg::Pose2D & msg)
{
    auto message_micon = std_msgs::msg::Float64MultiArray();

    x_r = msg.x;
    y_r = msg.y;
    theta_r = msg.theta;

    getVelocity(x_r, y_r, theta_r, theta_l_pre, high_or_low);

    message_micon.data = {shootVelocity[1], shootVelocity[2]};

    publisher_micon->publish(message_micon);

    RCLCPP_INFO(this->get_logger(), "firing parameters: [pitch: %f, yaw: %f]", shootVelocity[1], shootVelocity[2]);
}

//コントローラ入力を定期的にsubしodrive_cmdにpub
void Shoot::topic_callback_controller(const sensor_msgs::msg::Joy & msg)
{
    commence_fire = msg.buttons[0];
    if(commence_fire)
    {
        auto message_odrive = rogidrive_msg::msg::RogidriveMessage();
        message_odrive.name = "motor1";
        message_odrive.mode = 2;
        torque = (shooter::radious / shooter::barrel_length) * ((shooter::m*shootVelocity[0]*shootVelocity[0] / 2) + shooter::barrel_length*sin(shootVelocity[1]) ) * k_torque;
        message_odrive.current = torque / torque_constant;
        publisher_odrive->publish(message_odrive);

        RCLCPP_INFO(this->get_logger(), "firing parameters: [speed: %f]", shootVelocity[0]);
    }
}

//回転し過ぎないようodriveからエンコーダーを読んでpidで制御
void Shoot::topic_callback_odrive_pos(const rogidrive_msg::msg::RogidriveMultiArray & msg)
{
    motor_pos.clear();

    for(const auto &data : msg.data)
    {
        motor_pos.push_back(data.pos);
    }
    RCLCPP_INFO(this->get_logger(), "Stored %zu motor positions.", motor_pos.size());

    torque = pid(motor_pos[0], limit_pos, 0.4, 0, 0);

    if(motor_pos[0] == limit_pos)
    {
        pid_reset();
    }
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Shoot>());
  rclcpp::shutdown();
  return 0;
}