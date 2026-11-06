#pragma once
#include <cstdio>

#include "rvd/rail_io.hpp"

namespace rvd {
using pfcr::Vec3;
void write_csv_header(std::FILE* log);
void write_csv_row(std::FILE* log, int k, double t_tick, double t_wake, const Measurement& m, double age,
                   double dt_acq, double comp_shift, double track_err, const pfcr::StepResult& r,
                   const Command& cmd, double t_ctrl_start, double t_ctrl_end, const CommandAck& ack,
                   double z, int zi, const char* status, const std::string& reason);
}  // namespace rvd
