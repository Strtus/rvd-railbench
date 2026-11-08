#include "rvd/telemetry.hpp"

#include <stdexcept>
namespace rvd {
static void put(std::FILE* f, double v) {
  if (has(v))
    std::fprintf(f, "%.6f,", v);
  else
    std::fputs(",", f);
}

void write_csv_header(std::FILE* log) {
  std::fprintf(
      log,
      "k,t_tick,t_wake,t_acquired,t_received,t_user1,t_user2,sensor_raw_stamp,frame_id,meas_age_s,dt_acq_s,"
      "rail_x,rail_y,rail_z,comp_shift_m,track_err_m,hx,hy,hz,hvx,hvy,hvz,a0,a1,a2,ud0,ud1,ud2,ua0,ua1,ua2,"
      "ur0,ur1,ur2,nx,ny,nz,nvx,nvy,nvz,tx,ty,tz,tvx,tvy,tvz,t_ctrl_start,t_ctrl_end,t_sent,t_plc_ack,"
      "t_user3,t_user4,plc_raw_stamp,backup,intervened,obs_max_z,obs_max_z_idx,status,reason,"
      "rx,ry,rz,enc_x,enc_y,enc_z,t_encoder\n");
}
void write_csv_row(std::FILE* log, int k, double t_tick, double t_wake, const Measurement& m, double age,
                   double dt_acq, double comp_shift, double track_err, const pfcr::StepResult& r,
                   const Command& cmd, double t_ctrl_start, double t_ctrl_end, const CommandAck& ack,
                   double z, int zi, const char* status, const std::string& reason) {
  std::fprintf(log, "%d,", k);
  for (double v : {t_tick, t_wake, m.t_acquired, m.t_received, m.t_user1, m.t_user2, m.sensor_raw_stamp})
    put(log, v);
  std::fprintf(log, "%lld,", m.frame_id);
  for (double v : {age, dt_acq}) put(log, v);
  for (double v : m.rail_pos) std::fprintf(log, "%.6f,", v);
  for (double v : {comp_shift, track_err}) std::fprintf(log, "%.6f,", v);
  for (double v : r.state) std::fprintf(log, "%.6f,", v);
  for (float v : r.action) std::fprintf(log, "%.6f,", v);
  for (const Vec3* u : {&r.u_des, &r.u_act, &r.u_real})
    for (double v : *u) std::fprintf(log, "%.6f,", v);
  for (double v : r.next_state) std::fprintf(log, "%.6f,", v);
  for (double v : cmd.rail_target) std::fprintf(log, "%.6f,", v);
  for (double v : cmd.rail_velocity) std::fprintf(log, "%.6f,", v);
  for (double v :
       {t_ctrl_start, t_ctrl_end, ack.t_sent, ack.t_plc_ack, ack.t_user3, ack.t_user4, ack.plc_raw_stamp})
    put(log, v);
  std::fprintf(log, "%d,%d,%.2f,%d,%s,%s,", int(r.backup_active), int(r.intervened), z, zi, status,
               reason.c_str());
  for (double v : cmd.rail_reference) put(log, v);
  for (double v : m.encoder_pos) put(log, v);
  if (has(m.t_encoder))
    std::fprintf(log, "%.6f\n", m.t_encoder);
  else
    std::fputs("\n", log);
  if (std::fflush(log) != 0 || std::ferror(log)) throw std::runtime_error("telemetry write failed");
}
}  // namespace rvd
