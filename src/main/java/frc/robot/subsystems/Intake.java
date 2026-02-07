/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.CANID.INTAKE_ID;
import static frc.robot.Constants.RobotConstants.MAX_BATTERY_VOLTAGE;
import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.BRAKE;

import com.nrg948.dashboard.annotations.DashboardCommand;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.nrg948.dashboard.annotations.DashboardPIDController;
import com.nrg948.dashboard.annotations.DashboardRadialGauge;
import com.nrg948.dashboard.annotations.DashboardTextDisplay;
import com.nrg948.dashboard.model.DataBinding;
import com.nrg948.preferences.PIDControllerPreference;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.parameters.MotorParameters;
import frc.robot.util.MotorController;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;

@DashboardDefinition
public class Intake extends SubsystemBase implements ActiveSubsystem {

  private static final MotorParameters MOTOR = MotorParameters.KrakenX60;
  private static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
  private static final double GEAR_RATIO = 1;
  private static final double METERS_PER_REVOLUTION = (WHEEL_DIAMETER * Math.PI) / GEAR_RATIO;
  private static final double MAX_VELOCITY = MOTOR.getFreeSpeedRPM() * METERS_PER_REVOLUTION / 60;
  private static final double INTAKE_VELOCITY = 5.0;
  private static final double OUTTAKE_VELOCITY = -2.5;

  private final MotorController motor =
      MOTOR.newController(
          "/Intake/Motor", INTAKE_ID, CLOCKWISE_POSITIVE, BRAKE, METERS_PER_REVOLUTION);

  private final RelativeEncoder encoder = motor.getEncoder();

  private final double KS = MOTOR.getKs();
  private final double KV = (MAX_BATTERY_VOLTAGE - KS) / MAX_VELOCITY;

  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(KS, KV);

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
  private final PIDControllerPreference pidController =
      new PIDControllerPreference("Intake", "PID Controller", 1, 0, 0);

  /** Creates a new Intake subsystem. */
  public Intake() {}

  public void setGoalVelocity(double goalVelocity) {
    this.goalVelocity = goalVelocity;
  }

  /** Intakes fuel */
  public void intake() {
    setGoalVelocity(INTAKE_VELOCITY);
  }

  /** Outtakes fuel */
  public void outtake() {
    setGoalVelocity(OUTTAKE_VELOCITY);
  }

  @Override
  public void disable() {
    goalVelocity = 0;
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    updateTelemetry();

    if (goalVelocity != 0) {
      double feedforward = feedForward.calculate(goalVelocity);
      double feedback = pidController.calculate(currentVelocity, goalVelocity);
      double voltage = feedforward + feedback;
      motor.setVoltage(voltage);

    } else {
      motor.stopMotor();
    }
  }

  private void updateTelemetry() {
    currentVelocity = encoder.getVelocity();
    motor.logTelemetry();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    motor.setIdleMode(idleMode);
  }

  /** Returns the intake's current velocity. */
  @DashboardTextDisplay(
      title = "Current Velocity (m/s)",
      column = 0,
      row = 3,
      width = 2,
      height = 1)
  public double getCurrentVelocity() {
    return currentVelocity;
  }

  /** Returns the intake's goal velocity. */
  public double getGoalVelocity() {
    return goalVelocity;
  }
}
