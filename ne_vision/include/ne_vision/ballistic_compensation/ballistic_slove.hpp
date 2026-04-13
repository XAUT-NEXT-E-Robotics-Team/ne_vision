#ifndef MODEL_HPP
#define MODEL_HPP
#include "ne_vision/ballistic_compensation/model_param.hpp"
#include <cmath>
#include <memory.h>
#include <functional>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace YUKINO
{
class BallisticModel
{
public:
  BallisticModel();
  ~BallisticModel();
  float   Cal_OssneHeight(float x,
                          float cal_pitch); // x 为击打距离，即为aim_distance
  Com_ps* judgeK1();
  float   get_ft(float x, float cal_pitch);
  // 需要注意的一点是弹丸并不是从云台直接的发射出去的，需要加上从云台到枪口的偏移（x方向为水平方向，Ossne为垂直方向）
  float Cal_TargetposPitch(float x,
                           float OssneHeight,
                           float x_offset,
                           float Ossne_offset); // 计算pitch

  // 新增：根据pitch和飞行时间计算弹丸在相机/云台坐标系下的3D位置，通过参数pos输出
  void Cal_BulletPosition(float            pitch,
                          float            time_s,
                          float            x_offset,
                          Eigen::Vector3d& pos);
};
} // namespace YUKINO
#endif
