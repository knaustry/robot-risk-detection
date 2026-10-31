// Persistent client for the Gazebo set_pose service (same service as `ign service`,
// without the ~1 s CLI start-up). stdin lines: "<model> <x> <y> <yaw>".
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <cmath>
#include <iostream>
#include <string>
int main() {
  ignition::transport::Node node;
  std::string name; double x, y, yaw;
  while (std::cin >> name >> x >> y >> yaw) {
    ignition::msgs::Pose req; ignition::msgs::Boolean rep; bool ok = false;
    req.set_name(name);
    req.mutable_position()->set_x(x); req.mutable_position()->set_y(y); req.mutable_position()->set_z(0);
    req.mutable_orientation()->set_z(std::sin(yaw / 2)); req.mutable_orientation()->set_w(std::cos(yaw / 2));
    bool done = node.Request("/world/robot_risk_world/set_pose", req, 500, rep, ok);
    std::cout << (done && ok ? "ok" : "fail") << std::endl;
  }
}
