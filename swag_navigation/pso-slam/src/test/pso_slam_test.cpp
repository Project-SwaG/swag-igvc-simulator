//#include "pso_slam/cell.h"
//#include "pso_slam/frame.h"
#include <chrono>
#include <eigen3/Eigen/Core>
#include <iostream>

#define SAVE_DATA_TO_FILE true
#define CELL_SIZE .4
#define SIDE_M 200

using namespace Eigen;
using std::cout;
using std::endl;

static Vector3d global_trans, previous_pose, trans_estimate, initial_trans;

// static Frame* current_frame;
// static Frame ref_frame(Vector3d::Zero(), 50, 50, CELL_SIZE);
//

int main(int argc, char **argv) {

  // current_frame = new Frame(initial_trans, SIDE_M, SIDE_M, SIDE_M, true);
  return 0;
}
