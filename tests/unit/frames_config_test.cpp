#include <filesystem>
#include <fstream>
#include <limits>

#include "rvd/frames.hpp"
#include "rvd/runtime.hpp"
#include "test_support.hpp"
int main(int argc, char** argv) {
  CHECK(argc == 2);
  rvd::RailConfig c;
  rvd::FrameMap default_map(c);
  CHECK((default_map.hill_to_rail({18, .3, -.2}) == pfcr::Vec3{18, .03, -.020000000000000004}));
  c.rail_axis[0] = 2;
  c.rail_axis[1] = 0;
  c.rail_axis[2] = 1;
  c.sign[0] = -1;
  c.scale[0] = 2;
  c.scale[1] = .25;
  c.scale[2] = 4;
  c.target_rail[0] = 1;
  c.target_rail[1] = .1;
  c.target_rail[2] = -.2;
  const rvd::FrameMap m(c);
  auto r = m.hill_to_rail({.1, 4, -.1});
  CHECK(near(r[0], 2) && near(r[1], -.3) && near(r[2], -.4));
  for (const auto& v : {pfcr::Vec3{0, 0, 0}, pfcr::Vec3{18, .3, -.2}, pfcr::Vec3{1e-12, -1e4, 4}}) {
    auto restored = m.rail_to_hill(m.hill_to_rail(v));
    for (int i = 0; i < 3; ++i) CHECK(near(v[i], restored[i]));
  }
  c.scale[0] = 0;
  rejects([&] { rvd::FrameMap bad(c); });
  c.scale[0] = 2;
  c.rail_axis[1] = 2;
  rejects([&] { rvd::validate(c); });
  c.rail_axis[1] = 0;
  c.sign[0] = 0;
  rejects([&] { rvd::validate(c); });
  c.sign[0] = -1;
  c.period_s = .5;
  rejects([&] { rvd::validate(c); });
  c.period_s = 1;
  c.thruster.tau = 1;
  rejects([&] { rvd::validate(c); });
  rvd::Options o;
  const std::string root = argv[1];
  for (const auto* name : {"rail", "pfcr_dqp", "simulation"})
    rvd::load_config(root + "/configs/" + name + ".ini", o);
  rvd::validate(o);
  auto temp = std::filesystem::path("config-test.ini");
  {
    std::ofstream f(temp);
    f << rvd::configuration_snapshot(o);
  }
  rvd::Options roundtrip;
  rvd::load_config(temp.string(), roundtrip);
  CHECK(rvd::configuration_snapshot(o) == rvd::configuration_snapshot(roundtrip));
  rvd::Options lead = o;
  {
    std::ofstream f(temp);
    f << "[rail]\n"
         "command_mode = lead\n"
         "lead_s = 0.4\n"
         "tracking_gain_per_s = 0.8\n"
         "sensor_disagreement_m = 0.02\n"
         "[simulation]\n"
         "mode = plc\n"
         "plc_acceleration_m_s2 = 0.1,0.2,0.3\n"
         "plc_delay_s = 0.05\n"
         "repeat_pose_at = 7\n"
         "encoder = false\n";
  }
  rvd::load_config(temp.string(), lead);
  rvd::validate(lead);
  CHECK(lead.rail.command_mode == rvd::CommandMode::Lead && lead.rail.lead_s == .4 &&
        lead.rail.tracking_gain_per_s == .8 && lead.rail.max_sensor_disagreement_m == .02);
  CHECK(lead.plc && lead.bounded && lead.plc_accel[2] == .3 && lead.plc_delay == .05 &&
        lead.sim_repeat_pose_at == 7 && !lead.sim_encoder);
  {
    std::ofstream f(temp);
    f << rvd::configuration_snapshot(lead);
  }
  rvd::Options lead_roundtrip;
  rvd::load_config(temp.string(), lead_roundtrip);
  CHECK(rvd::configuration_snapshot(lead) == rvd::configuration_snapshot(lead_roundtrip));
  lead.rail.tracking_gain_per_s = 1.5;
  rejects([&] { rvd::validate(lead); });
  lead.rail.tracking_gain_per_s = .8;
  lead.rail.lead_s = 0;
  rejects([&] { rvd::validate(lead); });
  lead.rail.lead_s = .4;
  lead.plc_delay = 1;
  rejects([&] { rvd::validate(lead); });
  lead.plc_delay = .05;
  lead.bounded = false;
  rejects([&] { rvd::validate(lead); });
  lead.plc = false;
  lead.sim_trace = "trace.csv";
  rejects([&] { rvd::validate(lead); });
  for (const auto* text :
       {"[rail]\nvelocity_m_s = 1,2\n", "[rail]\ntracking_error_m = nan\n", "[rail]\nunknown = 1\n",
        "[rail]\ntracking_error_m = 1\ntracking_error_m = 2\n", "[mapping]\naxis = 0,1,0.5\n",
        "[simulation]\nseed = -1\n", "[invalid]\nx = 1\n", "[rail]\ncommand_mode = fast\n",
        "[simulation]\nmode = turbo\n", "[simulation]\nencoder = 1\n"}) {
    {
      std::ofstream f(temp);
      f << text;
    }
    rejects([&] { rvd::load_config(temp.string(), roundtrip); });
  }
  std::filesystem::remove(temp);
  o.noise = -1;
  rejects([&] { rvd::validate(o); });
  o.noise = 0;
  o.lab = true;
  rejects([&] { rvd::validate(o); });
  o.lab = false;
  o.start_hill[0] = 21;
  rejects([&] { rvd::validate(o); });
}
