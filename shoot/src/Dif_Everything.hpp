const float g = 9.8;    //重力加速度(m/(s*s))
const float h = 2.43;    //ゴールの高さ(m)
const float x_g = 0;  //ゴールのx座標(m)
const float y_g = 7.475;  //ゴールのy座標(m)

const float l_1 = 0.2;  //自己位置が示す機体の部位から砲塔ヨー軸までの距離(メートル)
const float l_2 = 0.4;  //砲塔ヨー軸から射出口までの距離(メートル)

const float v_max = 10;    //射出装置の最大初速

const float radius = 0.05;  //回転軸からの距離（メートル）

const float k_rot = 1;  //射出速度調整さん（必要そうだから作ってみた）