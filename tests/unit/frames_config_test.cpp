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
  for (const auto* text :
       {"[rail]\nvelocity_m_s = 1,2\n", "[rail]\ntracking_error_m = nan\n", "[rail]\nunknown = 1\n",
        "[rail]\ntracking_error_m = 1\ntracking_error_m = 2\n", "[mapping]\naxis = 0,1,0.5\n",
        "[simulation]\nseed = -1\n", "[invalid]\nx = 1\n"}) {
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
