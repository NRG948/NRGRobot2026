/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.MAX_BATTERY_VOLTAGE;
import static frc.robot.RobotPreferences.FEED_VELOCITY;
import static frc.robot.RobotPreferences.UNFEED_VELOCITY;

import com.nrg948.actuator.MotorController;
import com.nrg948.actuator.MotorDirection;
import com.nrg948.actuator.MotorIdleMode;
import com.nrg948.actuator.Motors;
import com.nrg948.dashboard.annotations.DashboardCommand;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.nrg948.dashboard.annotations.DashboardPIDController;
import com.nrg948.dashboard.annotations.DashboardRadialGauge;
import com.nrg948.dashboard.annotations.DashboardTextDisplay;
import com.nrg948.dashboard.model.DataBinding;
import com.nrg948.preferences.PIDControllerPreference;
import com.nrg948.sensor.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@DashboardDefinition
public final class Rollers extends SubsystemBase implements ActiveSubsystem {
  // Rollers class includes rollers from hopper and indexer

  private static final DataLog LOG = DataLogManager.getLog();

  private static final Motors MOTOR = Motors.KrakenX60;

  private static final double EFFICIENCY = 0.9;

  private final double maxVelocity;
  private final MotorController motor;
  private final RelativeEncoder encoder;

  private final double KS = MOTOR.getKs();
  private final double KV;
  private final SimpleMotorFeedforward feedforward;

  @DashboardTextDisplay(title = "Goal Velocity (m/s)", column = 0, row = 2, width = 2, height = 1)
  private double goalVelocity = 0;

  @DashboardRadialGauge(
      title = "Current Velocity (m/s)",
      column = 0,
      row = 0,
      width = 2,
      height = 2,
      min = -15.9593,
      max = 15.9593)
  private double currentVelocity = 0;

  @DashboardTextDisplay(
      title = "Test Goal Velocity (m/s)",
      column = 2,
      row = 0,
      width = 2,
      height = 1,
      dataBinding = DataBinding.READ_WRITE,
      showSubmitButton = true)
  private double testGoalVelocity = 0;

  @DashboardCommand(
      title = "Set Test Goal Velocity",
      column = 2,
      row = 1,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command setTestGoalVelocityCommand =
      Commands.runOnce(() -> setGoalVelocity(testGoalVelocity), this)
          .withName("Set Test Goal Velocity");

  @DashboardCommand(
      title = "Disable",
      column = 2,
      row = 2,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command disableCommands =
      Commands.runOnce(this::disable, this).ignoringDisable(true).withName("Disable");

  @DashboardPIDController(title = "PID Controller", column = 4, row = 0, width = 2, height = 3)
  private final PIDControllerPreference pidController;

  private final DoubleLogEntry logCurrentVelocity;
  private final DoubleLogEntry logGoalVelocity;

  /** Creates a new Rollers subsystem. */
  public Rollers(String name, int motorId, double metersPerRevolution) {
    setName(name);
    maxVelocity = MOTOR.getFreeSpeedRPM() * metersPerRevolution / 60 * EFFICIENCY;
    KV = (MAX_BATTERY_VOLTAGE - KS) / maxVelocity;
    feedforward = new SimpleMotorFeedforward(KS, KV);
    motor =
        MOTOR.newController(
            "/" + name + "/Motor",
            motorId,
            MotorDirection.CLOCKWISE_POSITIVE,
            MotorIdleMode.BRAKE,
            metersPerRevolution);
    encoder = motor.getEncoder();
    pidController = new PIDControllerPreference(name, "PID Controller", 1, 0, 0);

    logCurrentVelocity = new DoubleLogEntry(LOG, name + "/Current Velocity");
    logGoalVelocity = new DoubleLogEntry(LOG, name + "/Goal Velocity");
  }

  /** Sets goal velocity for rollers. */
  public void setGoalVelocity(double goalVelocity) {
    this.goalVelocity = goalVelocity;
  }

  /** Feeds balls into rollers. */
  public void feed() {
    setGoalVelocity(FEED_VELOCITY.getValue());
  }

  /** Outfeeds balls into rollers. */
  public void unfeed() {
    setGoalVelocity(UNFEED_VELOCITY.getValue());
  }

  @Override
  public void disable() {
    goalVelocity = 0;
    motor.stopMotor();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    motor.setIdleMode(idleMode);
  }

  @Override
  public boolean isEnabled() {
    return goalVelocity != 0;
  }

  @Override
  public void periodic() {
    updateTelemetry();

    if (goalVelocity != 0) {
      double feedforward = this.feedforward.calculate(goalVelocity);
      double feedback = pidController.calculate(currentVelocity, goalVelocity);
      double voltage = feedforward + feedback;
      motor.setVoltage(voltage);

    } else {
      motor.setVoltage(0);
    }
  }

  private void updateTelemetry() {
    motor.logTelemetry();

    currentVelocity = encoder.getVelocity();
    logCurrentVelocity.append(currentVelocity);
    logGoalVelocity.append(goalVelocity);
  }

  @DashboardTextDisplay(
      title = "Current Velocity (m/s)",
      column = 0,
      row = 3,
      width = 2,
      height = 1)
  public double getCurrentVelocity() {
    return currentVelocity;
  }
}
