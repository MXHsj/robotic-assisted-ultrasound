#include <array>
#include <cmath>
#include <iostream>

#define rad2deg 180 / 3.14

std::array<double, 9> rot_tar{
    {-0.3438, -0.5770, 0.7402, -0.9341, 0.2885, -0.286, -0.0918, -0.7627, -0.6387}};

std::array<double, 9> rot_home{{-0.024062954327523797, -0.9997104395513141, -0.00010621275608472814,
                                -0.9993318188401705, 0.024056764463824314, -0.027517048118005913,
                                0.02751163540446256, -0.0005559996853696569, -0.9996213286948822}};

std::array<double, 3> rot2rpy(const std::array<double, 9>& rot_mat) {
  // approach from https://www.learnopencv.com/rotation-matrix-to-euler-angles/
  std::array<double, 3> rpy = {{0.0, 0.0, 0.0}};  // roll, pitch, yaw
  double sy = std::sqrt(rot_mat[0] * rot_mat[0] + rot_mat[1] * rot_mat[1]);
  bool singular = sy < 1e-6;
  if (!singular) {
    rpy[0] = atan2(rot_mat[5], rot_mat[8]);
    rpy[1] = atan2(-rot_mat[2], sy);
    rpy[2] = atan2(rot_mat[1], rot_mat[0]);
  } else {
    rpy[0] = atan2(rot_mat[7], rot_mat[4]);
    rpy[1] = atan2(-rot_mat[2], sy);
    rpy[2] = 0;
  }
  return rpy;
}

class Myclass {
 public:
  Myclass(std::array<double, 3>& d1) : data1(d1){};
  inline std::array<double, 3> getData() { return data1; };

 private:
  std::array<double, 3> data1{};
};

int main() {
  auto rpy_home = rot2rpy(rot_home);
  auto rpy_tar = rot2rpy(rot_tar);
  std::cout << rpy_home[0] * rad2deg << " " << rpy_home[1] * rad2deg << " " << rpy_home[2] * rad2deg
            << std::endl;
  // std::cout << rpy_tar[0] * rad2deg << " " << rpy_tar[1] * rad2deg << " " << rpy_tar[2] * rad2deg
  //           << std::endl;
  Myclass mc1(rpy_home);
  Myclass mc2(rpy_tar);
  auto data1 = mc1.getData();
  auto data2 = mc2.getData();
  // std::cout << data[0] * rad2deg << std::endl;
  // std::cout << data1 - data2 << std::endl;

  return 0;
}