#include "xgo_ros/xgo.h"

int main(int argc, char **argv) {
  init(argc, argv, "xgo_control_node");
  NodeHandle node_handle;
  Rate loop_rate(500);
  XGO xgo(&node_handle);
  cout << "Initalization done." << endl;

  while (ok()) {
    spinOnce();
    xgo.readState();
    loop_rate.sleep();
  }
  return 0;
}
