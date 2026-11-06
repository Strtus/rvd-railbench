#include <filesystem>
#include <fstream>

#include "rvd/artifact.hpp"
#include "rvd/pfcr_adapter.hpp"
#include "test_support.hpp"
int main(int argc, char** argv) {
  CHECK(argc == 2);
  const auto scratch = std::filesystem::path("hash-test.bin");
  { std::ofstream f(scratch, std::ios::binary); }
  CHECK(rvd::sha256_file(scratch.string()) ==
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  {
    std::ofstream f(scratch, std::ios::binary);
    f << "abc";
  }
  CHECK(rvd::sha256_file(scratch.string()) ==
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  {
    std::ofstream f(scratch, std::ios::binary);
    f << std::string(1000000, 'a');
  }
  CHECK(rvd::sha256_file(scratch.string()) ==
        "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
  const auto dir = std::string(argv[1]) + "/policies/" + pfcr::params::kModelId;
  rvd::verify_artifact(dir, true);
  const auto corrupt_dir = std::filesystem::path("corrupt-policy-test");
  std::filesystem::create_directories(corrupt_dir);
  {
    std::ofstream f(corrupt_dir / "pfcr_policy_weights.bin", std::ios::binary);
    f << "corrupt";
  }
  rejects([&] { rvd::verify_artifact(corrupt_dir.string(), false); });
  std::filesystem::remove(corrupt_dir / "pfcr_policy_weights.bin");
  std::filesystem::remove(corrupt_dir);
  std::filesystem::remove(scratch);
  std::filesystem::copy_file(dir + "/pfcr_policy_weights.bin", scratch,
                             std::filesystem::copy_options::overwrite_existing);
  {
    std::fstream f(scratch, std::ios::binary | std::ios::in | std::ios::out);
    f.seekp(8);
    const unsigned dim = 35;
    f.write(reinterpret_cast<const char*>(&dim), 4);
  }
  rejects([&] { pfcr::make_builtin_policy(scratch.string()); });
  std::filesystem::remove(scratch);
  rvd::PfcrAdapter adapter{rvd::RailConfig{}};
  rvd::StateEstimate initial{rvd::Frame::Hill, {18, .3, -.2, 0, 0, 0}, 0};
  rejects([&] { adapter.reset(initial); });
  adapter.initialize({dir, false});
  CHECK(adapter.status() == rvd::ControllerStatus::Ready);
  rejects([&] { adapter.update({initial, {}}); });
  adapter.reset(initial);
  rejects([&] { adapter.update({}); });
  auto wrong = initial;
  wrong.frame = rvd::Frame::Rail;
  rejects([&] { adapter.update({wrong, {}}); });
  pfcr::Controller original(pfcr::make_builtin_policy(dir + "/pfcr_policy_weights.bin"),
                            rvd::RailConfig{}.thruster, pfcr::VelocitySource::Model, 1.0);
  original.reset(initial.state);
  for (int k = 0; k < 40; ++k) {
    const auto expected = original.step({initial.state[0], initial.state[1], initial.state[2]});
    auto got = adapter.update({initial, {}});
    CHECK(got.reference.has_value());
    const auto& r = adapter.last_result();
    CHECK(r.state == expected.state && r.obs == expected.obs && r.action == expected.action);
    CHECK(r.next_state == expected.next_state && r.u_act == expected.u_act && r.u_real == expected.u_real);
    CHECK(r.status == expected.status && r.reason == expected.reason && r.intervened == expected.intervened);
    initial.state = r.next_state;
    initial.acquired_s += 1;
  }
  initial.state = {18, .3, -.2, 0, 0, 0};
  adapter.reset(initial);
  auto a = adapter.update({initial, {}});
  adapter.reset(initial);
  auto b = adapter.update({initial, {}});
  CHECK(a.reference->state == b.reference->state);
  rejects([&] { adapter.initialize({"nonexistent-policy", false}); });
  CHECK(adapter.status() == rvd::ControllerStatus::Uninitialized);
}
