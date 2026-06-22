#include "wbr_controller_v2.h"
#include "wbr_controller_v2/common/controller_config.h"
#include "wbr_controller_v2/common/controller_math.h"
#include "wbr_controller_v2/control/contact_safety.h"
#include "wbr_controller_v2/control/lqr_schedule.h"
#include "wbr_controller_v2/control/yaw_coordinator.h"
#include "wbr_controller_v2/control/wheel_allocator.h"
#include "wbr_controller_v2/model/leg_kinematics.h"
#include "wbr_controller_v2/model/state_estimator.h"

#include <cmath>

using namespace wbr::v2;

void WbrControllerV2::Apply(const mjModel* model, mjData* data,
                            double& target_leg_length, double& target_leg_angle) {
  if (!model || !data) {
    return;
  }
  telemetry_ = {};

  // Stage 1: sanitize UI commands and apply rate limits.
  target_leg_length = Clamp(target_leg_length, kMinTargetLegLength, kTargetHRadius);
  target_leg_angle = Clamp(target_leg_angle, -kMaxTargetLegAngle, kMaxTargetLegAngle);
  const bool time_reset = lqr_initialized_ && data->time < last_time_;
  double control_dt = model->opt.timestep;
  if (lqr_initialized_ && !time_reset && data->time > last_time_) {
    control_dt = Clamp(data->time - last_time_, 1e-4, 0.02);
  }
  target_linear_velocity_ = Clamp(target_linear_velocity_,
                                  -kMaxLinearVelocity, kMaxLinearVelocity);
  target_yaw_rate_ = Clamp(target_yaw_rate_, -kMaxYawRate, kMaxYawRate);
  const double safe_target_yaw_rate = Clamp(
      target_yaw_rate_, -kSustainedYawRateLimit, kSustainedYawRateLimit);
  commanded_linear_velocity_ = MoveTowards(
      commanded_linear_velocity_, target_linear_velocity_,
      kLinearAccelerationLimit * control_dt);
  const bool yaw_rate_braking =
      std::fabs(safe_target_yaw_rate) < std::fabs(commanded_yaw_rate_) ||
      safe_target_yaw_rate * commanded_yaw_rate_ < 0.0;
  commanded_yaw_rate_ = MoveTowards(
      commanded_yaw_rate_, safe_target_yaw_rate,
      (yaw_rate_braking ? kYawCommandDeceleration
                        : kYawCommandAcceleration) * control_dt);
  telemetry_.commanded_linear_velocity = commanded_linear_velocity_;
  telemetry_.commanded_yaw_rate = commanded_yaw_rate_;

  // Stage 2: observe both five-bar legs and update target trajectories.
  const bool first_leg_available =
      act_q_b_ >= 0 && act_q_d_ >= 0 && joint_q_b_ >= 0 && joint_q_d_ >= 0;
  const bool second_leg_available =
      act_q_b_2_ >= 0 && act_q_d_2_ >= 0 && joint_q_b_2_ >= 0 && joint_q_d_2_ >= 0;
  LegKinematics leg_1{};
  LegKinematics leg_2{};
  const bool first_leg_valid = first_leg_available &&
      ComputeLegKinematics(model, data, joint_q_b_, joint_q_d_, branch_1_, 1.0, leg_1);
  const bool second_leg_valid = second_leg_available &&
      ComputeLegKinematics(model, data, joint_q_b_2_, joint_q_d_2_, branch_2_, -1.0, leg_2);

  if (first_leg_valid) {
    telemetry_.leg_length[0] = leg_1.length;
    telemetry_.leg_length_rate[0] = leg_1.length_rate;
    leg_speed_1_ += kLegSpeedFilter * (leg_1.length_rate - leg_speed_1_);
    leg_angle_speed_1_ +=
        kStateSpeedFilter * (leg_1.angle_rate - leg_angle_speed_1_);
  }
  if (second_leg_valid) {
    telemetry_.leg_length[1] = leg_2.length;
    telemetry_.leg_length_rate[1] = leg_2.length_rate;
    leg_speed_2_ += kLegSpeedFilter * (leg_2.length_rate - leg_speed_2_);
    leg_angle_speed_2_ +=
        kStateSpeedFilter * (leg_2.angle_rate - leg_angle_speed_2_);
  }

  if (!command_initialized_ && first_leg_valid && second_leg_valid) {
    commanded_leg_length_ = 0.5 * (leg_1.length + leg_2.length);
    commanded_leg_angle_ =
        std::atan2(std::sin(leg_1.angle) + std::sin(leg_2.angle),
                   std::cos(leg_1.angle) + std::cos(leg_2.angle));
    command_initialized_ = true;
  }
  commanded_leg_length_ =
      MoveTowards(commanded_leg_length_, target_leg_length,
                  kTargetLengthSlewRate * control_dt);
  commanded_leg_angle_ =
      MoveTowards(commanded_leg_angle_, target_leg_angle,
                  kTargetAngleSlewRate * control_dt);

  double total_wheel_torque = 0.0;
  double total_leg_angle_torque = 0.0;
  double total_yaw_torque = 0.0;
  double differential_leg_angle_torque = 0.0;
  double support_feedforward_1 = 0.0;
  double support_feedforward_2 = 0.0;
  bool effective_wheel_grounded[2] = {false, false};
  const bool lqr_state_valid =
      first_leg_valid && second_leg_valid && joint_root_ >= 0 && body_plate_ >= 0 &&
      body_wheel_1_ >= 0 && body_wheel_2_ >= 0;
  if (lqr_state_valid) {
    // Stage 3: fuse IMU, chassis, wheel and contact observations.
    const bool wheel_1_grounded =
        geom_floor_ >= 0 && geom_wheel_1_ >= 0 &&
        GeomsInContact(data, geom_floor_, geom_wheel_1_);
    const bool wheel_2_grounded =
        geom_floor_ >= 0 && geom_wheel_2_ >= 0 &&
        GeomsInContact(data, geom_floor_, geom_wheel_2_);

    StateEstimatorHandles estimator_handles;
    estimator_handles.body_plate = body_plate_;
    estimator_handles.body_wheel[0] = body_wheel_1_;
    estimator_handles.body_wheel[1] = body_wheel_2_;
    estimator_handles.joint_wheel[0] = joint_wheel_;
    estimator_handles.joint_wheel[1] = joint_wheel_2_;
    estimator_handles.geom_wheel[0] = geom_wheel_1_;
    estimator_handles.geom_wheel[1] = geom_wheel_2_;
    estimator_handles.sensor_imu_gyro = sensor_imu_gyro_;
    estimator_handles.sensor_imu_accelerometer = sensor_imu_accelerometer_;
    estimator_handles.sensor_imu_quaternion = sensor_imu_quaternion_;
    estimator_handles.sensor_wheel_speed[0] = sensor_wheel_speed_1_;
    estimator_handles.sensor_wheel_speed[1] = sensor_wheel_speed_2_;
    const bool raw_wheel_grounded[2] = {
        wheel_1_grounded, wheel_2_grounded};
    const StateEstimate estimate = state_estimator_.Update(
        model, data, estimator_handles, raw_wheel_grounded,
        control_dt, time_reset);
    const double imu_roll_rate = estimate.roll_rate;
    const double imu_pitch_rate = estimate.pitch_rate;
    const double imu_yaw_rate = estimate.yaw_rate;
    const double raw_wheel_speed_difference =
        estimate.wheel_speed_difference;
    const double wheel_odometry_forward_speed =
        estimate.wheel_odometry_x_speed;

    double x = estimate.x;
    double raw_x_speed = estimate.x_speed;
    const double raw_pitch_speed = imu_pitch_rate;
    const double phi = estimate.pitch;
    telemetry_.estimated_roll = estimate.roll;
    telemetry_.estimated_pitch = estimate.pitch;
    telemetry_.estimated_yaw = estimate.yaw;
    telemetry_.estimated_x = estimate.x;
    telemetry_.estimated_x_speed = estimate.x_speed;
    telemetry_.wheel_odometry_x_speed = estimate.wheel_odometry_x_speed;
    telemetry_.wheel_odometry_confidence =
        estimate.wheel_odometry_confidence;
    const double track_width = estimate.track_width;

    const ContactSafetyOutput contact = contact_safety_.Update(
        raw_wheel_grounded, estimate.roll, estimate.pitch, control_dt);
    contact_safety_state_ = contact.state;
    const bool wheel_1_effectively_grounded = contact.effective_grounded[0];
    const bool wheel_2_effectively_grounded = contact.effective_grounded[1];
    effective_wheel_grounded[0] = wheel_1_effectively_grounded;
    effective_wheel_grounded[1] = wheel_2_effectively_grounded;
    const bool single_support = contact.single_support;
    const bool contact_airborne = contact.airborne;
    if (contact.state_changed) {
      // Re-anchor when support changes so stale position debt is not carried
      // across a wheel-slip or airborne interval.
      x_reference_ = x;
      filtered_x_speed_ = raw_x_speed;
    }
    const double contact_authority_scale = contact.authority;
    telemetry_.contact_safety_state = contact_safety_state_;
    telemetry_.contact_authority_scale = contact_authority_scale;

    if (single_support || contact_airborne) {
      commanded_linear_velocity_ = MoveTowards(
          commanded_linear_velocity_, 0.0,
          kEmergencyLinearDeceleration * control_dt);
      commanded_yaw_rate_ = MoveTowards(
          commanded_yaw_rate_, 0.0,
          kYawCommandDeceleration * control_dt);
      telemetry_.commanded_linear_velocity = commanded_linear_velocity_;
      telemetry_.commanded_yaw_rate = commanded_yaw_rate_;
    }
    const bool balance_active =
        lqr_enabled_ &&
        !contact_airborne &&
        (wheel_1_effectively_grounded || wheel_2_effectively_grounded);
    telemetry_.wheel_grounded[0] = wheel_1_grounded;
    telemetry_.wheel_grounded[1] = wheel_2_grounded;
    telemetry_.balance_active = balance_active;
    if (balance_active) {
      const bool reset_lqr = !lqr_initialized_ || time_reset;
      if (reset_lqr) {
        x_reference_ = x;
        filtered_x_speed_ = raw_x_speed;
        filtered_pitch_speed_ = raw_pitch_speed;
        filtered_yaw_speed_ = imu_yaw_rate;
        filtered_wheel_speed_difference_ = raw_wheel_speed_difference;
        lqr_initialized_ = true;
      } else {
        filtered_x_speed_ +=
            kStateSpeedFilter * (raw_x_speed - filtered_x_speed_);
        filtered_pitch_speed_ +=
            kStateSpeedFilter * (raw_pitch_speed - filtered_pitch_speed_);
        const double yaw_rate_alpha =
            1.0 - std::exp(-control_dt / kYawRateFilterTimeConstant);
        filtered_yaw_speed_ +=
            yaw_rate_alpha * (imu_yaw_rate - filtered_yaw_speed_);
        filtered_wheel_speed_difference_ +=
            yaw_rate_alpha * (raw_wheel_speed_difference -
                              filtered_wheel_speed_difference_);
      }
      if (!yaw_initialized_ || time_reset) {
        yaw_reference_ = estimate.yaw;
        filtered_yaw_speed_ = imu_yaw_rate;
        previous_yaw_speed_ = filtered_yaw_speed_;
        filtered_yaw_acceleration_ = 0.0;
        filtered_wheel_speed_difference_ = raw_wheel_speed_difference;
        yaw_initialized_ = true;
      }
      if (!reset_lqr && !time_reset && !single_support) {
        // Keep the body-forward position reference active during pure spin.
        // Releasing it with x_reference=x permitted a constant sagittal drift,
        // which appears as a curved left-forward path while yawing.
        x_reference_ += commanded_linear_velocity_ * control_dt;
        x_reference_ = Clamp(x_reference_, x - kMaxPositionTrackingError,
                             x + kMaxPositionTrackingError);
        // A/D is a yaw-rate command, not an angle trajectory. While turning,
        // follow the measured heading so slip does not accumulate angle debt;
        // when the command reaches zero, the last measured heading is held.
        if (std::fabs(commanded_yaw_rate_) > 1e-5 ||
            std::fabs(target_yaw_rate_) > 1e-5) {
          yaw_reference_ = estimate.yaw;
        }
      }
    } else {
      lqr_initialized_ = false;
      filtered_x_speed_ = raw_x_speed;
      filtered_pitch_speed_ = raw_pitch_speed;
      filtered_yaw_speed_ = imu_yaw_rate;
      filtered_wheel_speed_difference_ = raw_wheel_speed_difference;
    }
    const double world_leg_angle_1 =
        std::remainder(leg_1.angle + phi, 2.0 * M_PI);
    const double world_leg_angle_2 =
        std::remainder(leg_2.angle + phi, 2.0 * M_PI);
    const double theta =
        std::atan2(std::sin(world_leg_angle_1) + std::sin(world_leg_angle_2),
                   std::cos(world_leg_angle_1) + std::cos(world_leg_angle_2));
    const double theta_rate =
        0.5 * (leg_1.angle_rate + leg_2.angle_rate) + raw_pitch_speed;

    telemetry_.differential_leg_angle_error = std::remainder(
        leg_1.angle - leg_2.angle, 2.0 * M_PI);
    telemetry_.differential_leg_angle_rate =
        leg_angle_speed_1_ - leg_angle_speed_2_;

    // Stage 4: update contact safety and yaw-authority coordination.
    telemetry_.wheel_normal_force[0] = ContactNormalForce(
        model, data, geom_floor_, geom_wheel_1_);
    telemetry_.wheel_normal_force[1] = ContactNormalForce(
        model, data, geom_floor_, geom_wheel_2_);
    YawCoordinatorInput yaw_input;
    yaw_input.reference_enabled = yaw_enabled_ &&
        contact_safety_state_ == WbrContactSafetyState::kDualSupport &&
        wheel_1_effectively_grounded && wheel_2_effectively_grounded;
    yaw_input.time_reset = time_reset;
    yaw_input.control_dt = control_dt;
    yaw_input.commanded_yaw_rate = commanded_yaw_rate_;
    yaw_input.measured_yaw_rate = filtered_yaw_speed_;
    yaw_input.split_angle = telemetry_.differential_leg_angle_error;
    yaw_input.split_rate = telemetry_.differential_leg_angle_rate;
    yaw_input.roll = estimate.roll;
    yaw_input.roll_rate = imu_roll_rate;
    yaw_input.pitch = phi;
    yaw_input.normal_force[0] = telemetry_.wheel_normal_force[0];
    yaw_input.normal_force[1] = telemetry_.wheel_normal_force[1];
    const YawCoordinatorOutput yaw_coord = yaw_coordinator_.Update(yaw_input);

    telemetry_.coordinated_yaw_rate = yaw_coord.coordinated_yaw_rate;
    telemetry_.differential_leg_angle_reference = yaw_coord.split_reference;
    telemetry_.differential_leg_angle_residual = yaw_coord.split_residual;
    telemetry_.differential_leg_angle_rate_residual =
        yaw_coord.split_rate_residual;
    telemetry_.predicted_split_error = yaw_coord.predicted_split_error;
    telemetry_.split_activity = yaw_coord.split_activity;
    telemetry_.predicted_roll = yaw_coord.predicted_roll;
    telemetry_.predicted_normal_force[0] =
        yaw_coord.predicted_normal_force[0];
    telemetry_.predicted_normal_force[1] =
        yaw_coord.predicted_normal_force[1];
    telemetry_.yaw_split_authority = yaw_coord.split_authority;
    telemetry_.yaw_attitude_authority = yaw_coord.attitude_authority;
    telemetry_.yaw_contact_authority = yaw_coord.contact_authority;
    telemetry_.yaw_authority_scale = yaw_coord.authority;
    if (wheel_1_grounded || wheel_2_grounded) {
      differential_leg_angle_torque = Clamp(
          -kDifferentialLegAngleKp *
              telemetry_.differential_leg_angle_error -
          kDifferentialLegAngleKd *
              telemetry_.differential_leg_angle_rate,
          -kDifferentialLegAngleTorqueLimit,
          kDifferentialLegAngleTorqueLimit);
    }
    const double support_alpha =
        1.0 - std::exp(-control_dt / kSupportFilterTimeConstant);
    support_factor_1_ +=
        support_alpha *
        ((wheel_1_grounded ? 1.0 : 0.0) -
         support_factor_1_);
    support_factor_2_ +=
        support_alpha *
        ((wheel_2_grounded ? 1.0 : 0.0) -
         support_factor_2_);
    const double half_weight =
        0.5 * mj_getTotalmass(model) * std::fabs(model->opt.gravity[2]);
    if (contact_safety_state_ == WbrContactSafetyState::kSingleSupportFirst) {
      support_feedforward_1 = half_weight /
          std::fmax(std::cos(world_leg_angle_1), 0.5);
      support_feedforward_2 = 0.0;
    } else if (contact_safety_state_ ==
               WbrContactSafetyState::kSingleSupportSecond) {
      support_feedforward_1 = 0.0;
      support_feedforward_2 = half_weight /
          std::fmax(std::cos(world_leg_angle_2), 0.5);
    } else {
      support_feedforward_1 =
          support_factor_1_ * half_weight /
          std::fmax(std::cos(world_leg_angle_1), 0.5);
      support_feedforward_2 =
          support_factor_2_ * half_weight /
          std::fmax(std::cos(world_leg_angle_2), 0.5);
    }

    // The sagittal LQR controls the common leg force. Stabilize the orthogonal
    // roll mode with differential axial force, whose sign and gain are
    // identified by lqr_open_loop_test on the free-root model.
    if (wheel_1_grounded && wheel_2_grounded && !single_support &&
        !contact_airborne) {
      const double roll = estimate.roll;
      const double roll_rate = imu_roll_rate;
      const double differential_force = Clamp(
          -kRollForceKp * roll - kRollForceKd * roll_rate,
          -kRollDifferentialForceLimit, kRollDifferentialForceLimit);
      support_feedforward_1 += 0.5 * differential_force;
      support_feedforward_2 -= 0.5 * differential_force;
    }

    if (time_reset) {
      leg_length_integral_[0] = 0.0;
      leg_length_integral_[1] = 0.0;
    }
    const LegKinematics* legs[2] = {&leg_1, &leg_2};
    const double support_feedforward[2] = {
        support_feedforward_1, support_feedforward_2};
    const double support_factor[2] = {support_factor_1_, support_factor_2_};
    for (int leg = 0; leg < 2; ++leg) {
      const bool integral_enabled = support_factor[leg] > 0.5;
      if (integral_enabled) {
        const double error = commanded_leg_length_ - legs[leg]->length;
        const double integral_limit =
            kLegIntegralForceLimit / kLegLengthKi;
        const double candidate = Clamp(
            leg_length_integral_[leg] + error * control_dt,
            -integral_limit, integral_limit);
        const double candidate_force =
            kLegLengthKp * error - kLegLengthKd *
                (leg == 0 ? leg_speed_1_ : leg_speed_2_) +
            support_feedforward[leg] + kLegLengthKi * candidate;
        const bool pushes_further_into_saturation =
            (candidate_force > kLegForceLimit && error > 0.0) ||
            (candidate_force < -kLegForceLimit && error < 0.0);
        if (!pushes_further_into_saturation) {
          leg_length_integral_[leg] = candidate;
        }
      } else {
        leg_length_integral_[leg] *= std::exp(-control_dt / 0.25);
      }
      telemetry_.integral_force[leg] =
          kLegLengthKi * leg_length_integral_[leg];
    }

    const double state_error[6] = {
        std::remainder(theta - commanded_leg_angle_, 2.0 * M_PI),
        theta_rate,
        x - x_reference_,
        raw_x_speed - commanded_linear_velocity_,
        phi,
        raw_pitch_speed,
    };
    for (int state = 0; state < 6; ++state) {
      telemetry_.state_error[state] = state_error[state];
    }
    if (balance_active) {
      // Stage 5: compose sagittal LQR, roll control and yaw requests.
      double scheduled_lqr_gain[2][6];
      InterpolateLqrGain(0.5 * (leg_1.length + leg_2.length),
                         scheduled_lqr_gain);
      for (int state = 0; state < 6; ++state) {
        total_wheel_torque -=
            scheduled_lqr_gain[0][state] * state_error[state];
        total_leg_angle_torque -=
            scheduled_lqr_gain[1][state] * state_error[state];
      }
      total_wheel_torque *= contact_authority_scale;
      total_leg_angle_torque *= contact_authority_scale;
      telemetry_.requested_wheel_torque = total_wheel_torque;

      // First form the yaw request from attitude/contact/split safety.  Wheel
      // torque availability is handled by the allocator below; feeding it
      // back here creates a deadlock where yaw cannot request the headroom it
      // needs in order to start.
      telemetry_.spin_mode_blend = Clamp(
          (std::fabs(commanded_yaw_rate_) - kSpinModeEntryYawRate) /
              (kSpinModeFullYawRate - kSpinModeEntryYawRate),
          0.0, 1.0);
      bool yaw_braking_request = false;
      if (yaw_enabled_ &&
          contact_safety_state_ == WbrContactSafetyState::kDualSupport &&
          wheel_1_grounded && wheel_2_grounded &&
          model->jnt_type[joint_root_] == mjJNT_FREE) {
        telemetry_.yaw_error = std::remainder(
            estimate.yaw - yaw_reference_, 2.0 * M_PI);
        telemetry_.yaw_rate = filtered_yaw_speed_;
        telemetry_.yaw_rate_error =
            yaw_coord.coordinated_yaw_rate - telemetry_.yaw_rate;
        const double raw_yaw_acceleration =
            (filtered_yaw_speed_ - previous_yaw_speed_) / control_dt;
        const double yaw_acceleration_alpha = 1.0 - std::exp(
            -control_dt / kYawAccelerationFilterTimeConstant);
        filtered_yaw_acceleration_ += yaw_acceleration_alpha *
            (raw_yaw_acceleration - filtered_yaw_acceleration_);
        previous_yaw_speed_ = filtered_yaw_speed_;
        // The second wheel joint axis is mirrored.  Positive world yaw therefore
        // corresponds to a negative r*(omega_1 + omega_2) measurement.
        telemetry_.target_wheel_speed_difference =
            -track_width * yaw_coord.coordinated_yaw_rate;
        telemetry_.wheel_speed_difference =
            filtered_wheel_speed_difference_;
        const double yaw_drive_torque =
            kYawRateKp * telemetry_.yaw_rate_error -
            kYawAccelerationKd * filtered_yaw_acceleration_;
        yaw_braking_request =
            std::fabs(telemetry_.yaw_rate) > 0.02 &&
            yaw_drive_torque * telemetry_.yaw_rate < 0.0;
        // Keep wheel-speed difference as slip/kinematics telemetry only.  The
        // yaw-rate loop follows the SPR structure and closes exclusively on
        // IMU wz, avoiding wheel-contact and mirrored-axis errors in feedback.
        // The coordinator shapes the target before the PD. A square-root
        // safety gate remains on the resulting torque: it is mild at partial
        // authority, but removes braking torque completely at a hard limit.
        // Safety authority suppresses torque that would add yaw energy.  Keep
        // counter-yaw braking available when roll/split authority collapses;
        // otherwise the robot coasts through the safety boundary and tips.
        const double yaw_drive_authority = yaw_braking_request
            ? 1.0
            : std::sqrt(telemetry_.yaw_authority_scale);
        telemetry_.requested_yaw_torque =
            yaw_drive_authority * yaw_drive_torque;
        total_yaw_torque = Clamp(telemetry_.requested_yaw_torque,
                                 -kTotalYawTorqueLimit,
                                 kTotalYawTorqueLimit);
        telemetry_.applied_yaw_torque = total_yaw_torque;
      }

      // Jointly allocate the common balance torque and differential yaw
      // torque.  Spin mode reserves only what the current yaw request needs,
      // capped at 2 N.m/side; unsafe yaw posture immediately returns that
      // capacity to sagittal balance.
      const double demanded_yaw_reserve_per_wheel = Clamp(
          0.5 * kDecoupledYawWheelInputScale *
                  std::fmax(std::fabs(total_yaw_torque),
                            std::fabs(commanded_yaw_torque_)) +
              kSpinYawReserveBufferPerWheel,
          0.0, kSpinYawReservePerWheel);
      telemetry_.reserved_yaw_torque_per_wheel =
          demanded_yaw_reserve_per_wheel * telemetry_.spin_mode_blend *
          (yaw_braking_request ? 1.0 : telemetry_.yaw_authority_scale);
      const double balance_torque_limit = 2.0 * std::fmax(
          0.0, kPerWheelTorqueLimit -
                   telemetry_.reserved_yaw_torque_per_wheel);
      const double requested_balance_torque = total_wheel_torque;
      total_wheel_torque = Clamp(total_wheel_torque,
                                 -balance_torque_limit,
                                 balance_torque_limit);
      telemetry_.balance_torque_authority =
          std::fabs(requested_balance_torque) > 1e-9
              ? std::fabs(total_wheel_torque / requested_balance_torque)
              : 1.0;
      const double common_wheel_demand =
          0.5 * std::fabs(total_wheel_torque);
      telemetry_.wheel_torque_margin = std::fmax(
          0.0, kPerWheelTorqueLimit - common_wheel_demand);
      telemetry_.yaw_torque_authority = RiseAuthority(
          telemetry_.wheel_torque_margin,
          kYawTorqueMarginHard, kYawTorqueMarginSoft);
    }
    telemetry_.requested_leg_angle_torque = total_leg_angle_torque;
    total_wheel_torque = Clamp(total_wheel_torque, -kTotalWheelTorqueLimit,
                               kTotalWheelTorqueLimit);
    total_leg_angle_torque =
        Clamp(total_leg_angle_torque, -kTotalLegAngleTorqueLimit,
              kTotalLegAngleTorqueLimit);
    telemetry_.applied_wheel_torque = total_wheel_torque;
    telemetry_.applied_leg_angle_torque = total_leg_angle_torque;
  }

  const bool yaw_command_released = std::fabs(target_yaw_rate_) < 1e-5;
  const bool yaw_command_reversed =
      target_yaw_rate_ * commanded_yaw_torque_ < 0.0;
  const bool yaw_torque_braking =
      yaw_command_reversed ||
      (yaw_command_released &&
       std::fabs(total_yaw_torque) < std::fabs(commanded_yaw_torque_));
  commanded_yaw_torque_ = MoveTowards(
      commanded_yaw_torque_, total_yaw_torque,
      (yaw_torque_braking ? kYawTorqueBrakeRate : kYawTorqueRiseRate) *
          control_dt);
  if (contact_safety_state_ == WbrContactSafetyState::kSingleSupportFirst ||
      contact_safety_state_ == WbrContactSafetyState::kSingleSupportSecond ||
      contact_safety_state_ == WbrContactSafetyState::kAirborne) {
    commanded_yaw_torque_ = 0.0;
  }
  total_yaw_torque = commanded_yaw_torque_;

  // Stage 6: synchronize the identified yaw/leg-split actuator pair.
  differential_leg_angle_torque +=
      kDecoupledYawLegTorquePerCommand * total_yaw_torque;
  differential_leg_angle_torque = Clamp(
      differential_leg_angle_torque, -kDifferentialLegAngleTorqueLimit,
      kDifferentialLegAngleTorqueLimit);
  commanded_differential_leg_angle_torque_ = MoveTowards(
      commanded_differential_leg_angle_torque_, differential_leg_angle_torque,
      kDifferentialLegAngleTorqueSlewRate * control_dt);
  differential_leg_angle_torque = commanded_differential_leg_angle_torque_;
  if (contact_safety_state_ == WbrContactSafetyState::kSingleSupportFirst ||
      contact_safety_state_ == WbrContactSafetyState::kSingleSupportSecond ||
      contact_safety_state_ == WbrContactSafetyState::kAirborne) {
    commanded_differential_leg_angle_torque_ = 0.0;
    differential_leg_angle_torque = 0.0;
  }
  telemetry_.differential_leg_angle_torque = differential_leg_angle_torque;

  // Stage 7: map generalized leg commands through VMC to joint actuators.
  // The LQR's Tp uses the paper's body-on-leg torque convention.  The VMC
  // below applies leg-on-body generalized force, which is the opposite side
  // of that action/reaction pair (the identification path uses the same
  // conversion in ApplyLegController).  Convert conventions before J^T F.
  const double common_leg_angle_torque = -0.5 * total_leg_angle_torque;
  const double first_leg_angle_torque =
      common_leg_angle_torque + 0.5 * differential_leg_angle_torque;
  const double second_leg_angle_torque =
      common_leg_angle_torque - 0.5 * differential_leg_angle_torque;
  double first_leg_target_length = commanded_leg_length_;
  double second_leg_target_length = commanded_leg_length_;
  double desired_contact_leg_offset[2] = {0.0, 0.0};
  if (contact_safety_state_ == WbrContactSafetyState::kSingleSupportFirst) {
    desired_contact_leg_offset[0] = -kGroundedLegYield;
    desired_contact_leg_offset[1] = kAirborneLegSearchExtension;
  } else if (contact_safety_state_ ==
             WbrContactSafetyState::kSingleSupportSecond) {
    desired_contact_leg_offset[0] = kAirborneLegSearchExtension;
    desired_contact_leg_offset[1] = -kGroundedLegYield;
  }
  for (int leg = 0; leg < 2; ++leg) {
    contact_leg_length_offset_[leg] = MoveTowards(
        contact_leg_length_offset_[leg], desired_contact_leg_offset[leg],
        kContactLegOffsetSlewRate * control_dt);
  }
  first_leg_target_length = Clamp(
      commanded_leg_length_ + contact_leg_length_offset_[0],
      kMinTargetLegLength, kTargetHRadius);
  second_leg_target_length = Clamp(
      commanded_leg_length_ + contact_leg_length_offset_[1],
      kMinTargetLegLength, kTargetHRadius);
  if (first_leg_valid) {
    telemetry_.axial_force[0] =
        ApplyLegVmc(data, act_q_b_, act_q_d_, leg_1, first_leg_target_length,
                    support_feedforward_1, telemetry_.integral_force[0],
                    first_leg_angle_torque, 1.0,
                    leg_speed_1_);
  } else if (first_leg_available) {
    data->ctrl[act_q_b_] = 0.0;
    data->ctrl[act_q_d_] = 0.0;
  }
  if (second_leg_valid) {
    telemetry_.axial_force[1] =
        ApplyLegVmc(data, act_q_b_2_, act_q_d_2_, leg_2,
                    second_leg_target_length, support_feedforward_2,
                    telemetry_.integral_force[1], second_leg_angle_torque, -1.0,
                    leg_speed_2_);
  } else if (second_leg_available) {
    data->ctrl[act_q_b_2_] = 0.0;
    data->ctrl[act_q_d_2_] = 0.0;
  }

  if (act_wheel_ >= 0 && act_wheel_2_ >= 0) {
    // Stage 8: allocate balance/yaw requests to physical wheel actuators.
    WheelAllocationInput allocation_input;
    allocation_input.balance_torque = total_wheel_torque;
    allocation_input.yaw_torque = total_yaw_torque;
    allocation_input.grounded[0] = effective_wheel_grounded[0];
    allocation_input.grounded[1] = effective_wheel_grounded[1];
    allocation_input.contact_state = contact_safety_state_;
    const WheelAllocationOutput allocation =
        AllocateWheelTorque(allocation_input);
    WriteWheelActuators(data, act_wheel_, act_wheel_2_, allocation);
    telemetry_.applied_yaw_torque = allocation.applied_yaw_torque;
  }
  last_time_ = data->time;
}
