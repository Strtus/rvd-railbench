#include <cstdio>

#include "rvd/experiment.hpp"
#include "rvd/preview.hpp"
int main(int argc, char** argv) {
  if (argc == 5 && std::string(argv[1]) == "--preview") {
    try {
      rvd::write_preview(argv[2], argv[3], argv[4]);
      std::puts("offline comparison saved (no rail commands)");
      return 0;
    } catch (const std::exception& e) {
      std::fprintf(stderr, "preview failed: %s\n", e.what());
      return 2;
    }
  }
  if (argc < 2 || argc > 3) {
    std::fprintf(stderr,
                 "usage: rail_replay EXPERIMENT_DIR [MODEL_DIR]\n"
                 "       rail_replay --preview EXPERIMENT_DIR MODEL_DIR NEW_OUTPUT_DIR\n");
    return 2;
  }
  try {
    const auto r = rvd::replay(argv[1], argc == 3 ? argv[2] : "");
    std::printf("replayed %d controller inputs, maximum output error %.17g\n", r.steps, r.max_error);
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "replay failed: %s\n", e.what());
    return 2;
  }
}
