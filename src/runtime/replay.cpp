#include "rvd/experiment.hpp"
#include "rvd/pfcr_adapter.hpp"
#include "rvd/artifact.hpp"
#include <set>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
namespace rvd {
namespace {
std::vector<std::string> cells(std::string s) {
  if(!s.empty()&&s.back()=='\r')s.pop_back();
  std::vector<std::string> result;std::size_t pos=0;
  while(true) {const auto end=s.find(',',pos);result.push_back(s.substr(pos,end-pos));if(end==std::string::npos)break;pos=end+1;}
  return result;
}
double num(const std::string& s) {
  size_t n=0;const double x=std::stod(s,&n);
  if(n!=s.size()||!std::isfinite(x))throw std::runtime_error("invalid replay number");return x;
}
}
ReplaySummary replay(const std::string& directory,const std::string& model) {
  std::ifstream checks(directory+"/checksums.sha256");
  if (!checks) throw std::runtime_error("experiment has no completion checksums");
  std::string checksum_line;
  std::set<std::string> verified;
  const std::set<std::string> allowed{"configuration.ini", "metadata.json", "initial_state.csv", "controller.csv", "events.jsonl", "trajectory.csv"};
  while (std::getline(checks,checksum_line)) {
    if (!checksum_line.empty() && checksum_line.back()=='\r') checksum_line.pop_back();
    if (checksum_line.size()<67 || checksum_line.substr(64,2)!="  ") throw std::runtime_error("invalid checksum record");
    const auto name=checksum_line.substr(66);
    if (!allowed.count(name) || !verified.insert(name).second || sha256_file(directory+"/"+name)!=checksum_line.substr(0,64))
      throw std::runtime_error("experiment integrity mismatch: "+name);
  }
  if (checks.bad() || !verified.count("configuration.ini") || !verified.count("initial_state.csv") || !verified.count("controller.csv"))
    throw std::runtime_error("incomplete experiment checksums");
  Options options;load_config(directory+"/configuration.ini",options);validate(options);
  PfcrAdapter controller(options.rail);controller.initialize({model,options.onnx});
  std::ifstream initial(directory+"/initial_state.csv"),inputs(directory+"/controller.csv");
  if(!initial||!inputs)throw std::runtime_error("incomplete experiment inputs");
  std::string line;
  std::getline(initial,line);if(line!="x,y,z,vx,vy,vz")throw std::runtime_error("unsupported initial-state schema");
  std::getline(initial,line);auto init=cells(line);if(init.size()!=6)throw std::runtime_error("invalid initial-state row");
  StateEstimate state;for(int i=0;i<6;++i)state.state[i]=num(init[i]);controller.reset(state);
  std::getline(inputs,line);
  if(!line.empty()&&line.back()=='\r')line.pop_back();
  if(line!="k,t_acquired,hx,hy,hz,a0,a1,a2,n0,n1,n2,n3,n4,n5,backup,intervened,status,reason")
    throw std::runtime_error("unsupported replay schema");
  ReplaySummary summary;
  bool complete=false;
  while(std::getline(inputs,line)) {
    const auto row=cells(line);
    if (row.size()==2 && row[0]=="#complete") {
      if (num(row[1])!=summary.steps || std::getline(inputs,line)) throw std::runtime_error("invalid replay completion");
      complete=true;break;
    }
    if(row.size()!=18||num(row[0])!=summary.steps)throw std::runtime_error("invalid replay sequence");
    StateEstimate observation;observation.acquired_s=num(row[1]);
    for(int i=0;i<3;++i)observation.state[i]=num(row[2+i]);
    controller.update({observation,{}});const auto& r=controller.last_result();
    for(int i=0;i<3;++i)summary.max_error=std::max(summary.max_error,std::fabs(r.action[i]-num(row[5+i])));
    for(int i=0;i<6;++i)summary.max_error=std::max(summary.max_error,std::fabs(r.next_state[i]-num(row[8+i])));
    if(row[16]!=pfcr::status_name(r.status)||row[17]!=r.reason||num(row[14])!=int(r.backup_active)||num(row[15])!=int(r.intervened))
      throw std::runtime_error("replay status or safety flags differ at step "+row[0]);
    // Same backend/build should be exact; permit only representation-level differences.
    if(summary.max_error>1e-9)throw std::runtime_error("replay output differs at step "+row[0]);
    ++summary.steps;
  }
  if(!complete)throw std::runtime_error("incomplete controller recording");
  if(inputs.bad())throw std::runtime_error("replay read failed");
  return summary;
}
}  // namespace rvd
