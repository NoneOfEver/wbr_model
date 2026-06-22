#ifndef WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_CONTROL_LQR_SCHEDULE_H_
#define WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_CONTROL_LQR_SCHEDULE_H_

namespace wbr::v2 {

void InterpolateLqrGain(double leg_length, double gain[2][6]);

}  // namespace wbr::v2

#endif  // WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_CONTROL_LQR_SCHEDULE_H_
