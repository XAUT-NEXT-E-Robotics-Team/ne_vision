#include "ne_vision/ballistic_compensation/ballistic_slove.hpp"
#include <cstdio>

Com_ps  g_com_ps;
Com_ps* Com_ptr_ = &g_com_ps;

namespace YUKINO
{
BallisticModel::BallisticModel() {}
BallisticModel::~BallisticModel() {}
Com_ps* BallisticModel::judgeK1()
{
  if (Com_ptr_->ball_type == "small")
  {
    Com_ptr_->K1 = SMALL_BALL_K1;
  }
  else
  {
    Com_ptr_->K1 = BIG_BALL_K1;
  }
  // std::cout << Com_ptr_->K1 << std::endl;
  return Com_ptr_;
}
float BallisticModel::Cal_OssneHeight(float x, float cal_pitch)
{
  Com_ptr_->ft = (float)(exp(x * Com_ptr_->K1) - 1) /
                 (Com_ptr_->K1 * cos(cal_pitch) * Com_ptr_->muzzle_v);
  // std::cout<<Com_ptr_ -> ft << std::endl;
  float OssneHeight =
      (float)(sin(cal_pitch) * Com_ptr_->muzzle_v * Com_ptr_->ft) -
      0.5 * GRAVITY * pow(Com_ptr_->ft, 2);
  // std::cout<<OssneHeight <<std::endl;
  return OssneHeight;
}
float BallisticModel::get_ft(float x, float cal_pitch)
{
  Com_ptr_->ft = (float)(exp(x * Com_ptr_->K1) - 1) /
                 (Com_ptr_->K1 * cos(cal_pitch) * Com_ptr_->muzzle_v);
  return Com_ptr_->ft;
}
float BallisticModel::Cal_TargetposPitch(float x,
                                         float OssneHeight,
                                         float x_offset,
                                         float Ossne_offset)
{
  // 局部变量
  int   count = 0;
  float aim_ossne = OssneHeight;
  float cal_pitch = 0; // 迭代的初始角度为零
  float Drop_OssneHeight = 0;
  float Actual_error = 0;

  for (int i = 0; i < MAX_ITERATION_COUNT; i++)
  {
    cal_pitch = atan2(aim_ossne, x); // 计算角度
    // 考虑偏移
    Drop_OssneHeight = Cal_OssneHeight(
        x - (cos(cal_pitch) * x_offset - sin(cal_pitch) * Ossne_offset),
        cal_pitch);                                // 根据模型计算落点高度
    Actual_error = OssneHeight - Drop_OssneHeight; // 更新误差
    aim_ossne = aim_ossne + Actual_error * ITERATE_SCALE_FACTOR; // 误差补偿
    count++;
    if (fabs(Actual_error) < PRECISION)
    {
      break;
    }
    // printf("x = %f,原始pitch = %f,pitch = %f,迭代次数 = %d\n",
    //        x,
    //        -atan2(OssneHeight, x) * 180 / 3.14,
    //        -(cal_pitch * 180 / 3.14),
    //        count);
  }
  return -cal_pitch;
}

void BallisticModel::Cal_BulletPosition(float            pitch,
                                        float            time_s,
                                        float            x_offset,
                                        Eigen::Vector3d& pos)
{
  pos.setZero();
  if (std::abs(Com_ptr_->muzzle_v) < 0.1f || Com_ptr_->K1 < 1e-6f)
  {
    return;
  }

  // 弹道学公式：水平飞行距离 x_flight = ln(1 + t * K1 * v0 * cos(pitch)) / K1
  float flight_x =
      logf(1.0f + time_s * Com_ptr_->K1 * Com_ptr_->muzzle_v * cosf(pitch)) /
      Com_ptr_->K1;
  // 弹道学公式：竖直下落距离 z_flight = v0 * sin(pitch) * t - 0.5 * g * t^2
  float flight_z = Com_ptr_->muzzle_v * sinf(pitch) * time_s -
                   0.5f * GRAVITY * time_s * time_s;

  // 考虑弹丸并不是从云台直接发射，需加上云台到枪口的偏移(沿枪管方向x_offset)
  float x_total = flight_x + x_offset * cosf(pitch);
  float z_total = flight_z + x_offset * sinf(pitch);

  // 设为 Eigen(x,y,z) 只不过y项为0
  // （如果在云台/世界系，往往将水平面视为 X, 垂直平面视为 Z, 横向视为 Y）
  pos.x() = x_total;
  pos.y() = 0.0;
  pos.z() = z_total;
}
} // namespace YUKINO
