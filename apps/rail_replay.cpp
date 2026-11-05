#include "rvd/experiment.hpp"
#include <cstdio>
int main(int argc,char** argv) {
  if(argc<2||argc>3) {std::fprintf(stderr,"usage: rail_replay EXPERIMENT_DIR [MODEL_DIR]\n");return 2;}
  try {
    const auto r=rvd::replay(argv[1],argc==3?argv[2]:rvd::find_model_dir());
    std::printf("replayed %d controller inputs, maximum output error %.17g\n",r.steps,r.max_error);return 0;
  } catch(const std::exception& e) {std::fprintf(stderr,"replay failed: %s\n",e.what());return 2;}
}
