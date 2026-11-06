#include "rvd/simulation.hpp"

#include "test_support.hpp"
using namespace rvd;
int main() {
  Options o;
  o.fast = true;
  o.bounded = true;
  o.noise = 0;
  o.sim_latency = 0;
  SimulatedRailIO s({10, 0, 0}, o);
  Command cmd;
  cmd.t_arrive = 10;
  cmd.rail_target = {11, 0, 0};
  cmd.rail_velocity = {.1, 0, 0};
  CommandAck ack;
  CHECK(s.send_target(cmd, ack));
  CHECK(s.position()[0] == 10);
  double previous_v = 0;
  for (int k = 1; k <= 1000; ++k) {
    s.advance_to(k * .01);
    CHECK(std::fabs(s.velocity()[0]) <= o.rail.rail_vmax[0] + 1e-12);
    CHECK(std::fabs(s.velocity()[0] - previous_v) <= o.rail.rail_amax[0] * .01 + 1e-10);
    previous_v = s.velocity()[0];
    if (k == 10) CHECK(s.position()[0] < 10.01);
  }
  CHECK(near(s.position()[0], 11, 1e-5));
  CHECK(std::fabs(s.velocity()[0]) < .002);
  rejects([&] { s.advance_to(1); });
  o.bounded = false;
  SimulatedRailIO ideal({10, 0, 0}, o);
  CHECK(ideal.send_target(cmd, ack));
  CHECK(ideal.position()[0] == 11);
  o.noise = .001;
  SimulatedRailIO a({10, 0, 0}, o), b({10, 0, 0}, o);
  for (int k = 0; k < 20; ++k) {
    a.advance_to(k);
    b.advance_to(k);
    Measurement ma, mb;
    CHECK(a.read_measurement(ma) && b.read_measurement(mb));
    CHECK(ma.rail_pos == mb.rail_pos && ma.t_acquired == mb.t_acquired);
  }
  o.sim_freeze_at = 0;
  SimulatedRailIO frozen({10, 0, 0}, o);
  Measurement m1, m2;
  CHECK(frozen.read_measurement(m1));
  frozen.advance_to(1);
  CHECK(frozen.read_measurement(m2));
  CHECK(m1.frame_id == m2.frame_id && m1.t_acquired == m2.t_acquired && m2.t_received == 1);
  o.sim_fail_read_at = 0;
  SimulatedRailIO read_failure({10, 0, 0}, o);
  CHECK(!read_failure.read_measurement(m1));
  o.sim_fail_read_at = -1;
  o.sim_fail_send_at = 0;
  SimulatedRailIO send_failure({10, 0, 0}, o);
  CHECK(!send_failure.send_target(cmd, ack));
  o.sim_fail_send_at = -1;
  o.sim_freeze_at = -1;
  o.noise = 0;
  o.bounded = true;
  o.sim_latency = .1;
  SimulatedRailIO delayed({10, 0, 0}, o);
  CHECK(delayed.send_target(cmd, ack));
  delayed.advance_to(.2);
  CHECK(delayed.read_measurement(m1));
  CHECK(near(m1.t_acquired, .1));
  CHECK(m1.rail_pos[0] < delayed.position()[0]);
  const auto before = delayed.velocity()[0];
  delayed.hold();
  delayed.advance_to(.21);
  CHECK(std::fabs(delayed.velocity()[0] - before) <= o.rail.rail_amax[0] * .01 + 1e-12);
  CHECK(!delayed.send_target(cmd, ack));
}
