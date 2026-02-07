/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.CANID.SHOOTER_LOWER_LEFT_ID;
import static frc.robot.Constants.RobotConstants.CANID.SHOOTER_LOWER_RIGHT_ID;
import static frc.robot.Constants.RobotConstants.CANID.SHOOTER_UPPER_RIGHT_ID;
import static frc.robot.Constants.RobotConstants.MAX_BATTERY_VOLTAGE;
import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.COAST;

import com.nrg948.dashboard.annotations.DashboardCommand;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.nrg948.dashboard.annotations.DashboardPIDController;
import com.nrg948.dashboard.annotations.DashboardRadialGauge;
import com.nrg948.dashboard.annotations.DashboardTextDisplay;
import com.nrg948.dashboard.model.DataBinding;
import com.nrg948.preferences.PIDControllerPreference;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CANID;
import frc.robot.parameters.MotorParameters;
import frc.robot.util.MotorController;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;

@DashboardDefinition
public class Shooter extends SubsystemBase implements ActiveSubsystem {

  private static final DataLog LOG = DataLogManager.getLog();

  private static final MotorParameters SHOOTER_MOTOR = MotorParameters.KrakenX44;
  private static final double GEAR_RATIO = 1.0;
  private static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
  private static final double METERS_PER_REV = (WHEEL_DIAMETER * Math.PI) / GEAR_RATIO;

  @DashboardTextDisplay(title = "Max Velocity (m/s)", column = 0, row = 3, width = 2, height = 1)
  private static final double MAX_VELOCITY =
      SHOOTER_MOTOR.getFreeSpeedRPM() * METERS_PER_REV / 60.0;

  private static final double KS = SHOOTER_MOTOR.getKs();
  private static final double KV = (MAX_BATTERY_VOLTAGE - KS) / MAX_VELOCITY;

  // Maps distances from our alliance's hub into corresponding shooter velocities.
  private static final InterpolatingDoubleTreeMap SHOOTER_VELOCITIES = new InterpolatingDoubleTreeMap();

  static {
    SHOOTER_VELOCITIES.put(0.0, 0.0); // TODO: Test & fill out table
    SHOOTER_VELOCITIES.put(10.0, MAX_VELOCITY);
  }

  private final MotorController leftUpperMotor =
      SHOOTER_MOTOR.newController(
          "/Shooter/Left Upper Motor",
          CANID.SHOOTER_UPPER_LEFT_ID,
          CLOCKWISE_POSITIVE,
          COAST,
          METERS_PER_REV);

  private final MotorController rightUpperMotor =
      leftUpperMotor.createFollower(SHOOTER_UPPER_RIGHT_ID, true);
  private final MotorController rightLowerMotor =
      leftUpperMotor.createFollower(SHOOTER_LOWER_RIGHT_ID, true);
  private final MotorController leftLowerMotor =
      leftUpperMotor.createFollower(SHOOTER_LOWER_LEFT_ID, false);

  private final RelativeEncoder encoder = leftUpperMotor.getEncoder();

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV);

  @DashboardPIDController(title = "PID", column = 6, row = 0, width = 2, height = 3)
  private final PIDControllerPreference pidController =
      new PIDControllerPreference("Shooter", "PID Controller", 1, 0, 0);

  @DashboardTextDisplay(title = "Goal Velocity (m/s)", column = 0, row = 2, width = 2, height = 1)
  private double goalVelocity = 0;

  @DashboardRadialGauge(
      title = "Lower Velocity (m/s)",
      column = 0,
      row = 0,
      width = 2,
      height = 2,
      min = -41.270725699090676,
      max = 41.270725699090676)
  private double currentVelocity = 0;

  @DashboardTextDisplay(
      title = "Test Velocity (m/s)",
      column = 4,
      row = 0,
      width = 2,
      height = 1,
      dataBinding = DataBinding.READ_WRITE,
      showSubmitButton = true)
  private double testGoalVelocity = 0;

  @DashboardCommand(
      title = "Set Test Velocities (m/s)",
      column = 4,
      row = 2,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command setTestGoalVelocitiesCommand =
      Commands.runOnce(() -> setGoalVelocity(testGoalVelocity), this)
          .withName("Set Test Velocities");

  @DashboardCommand(
      title = "Disable",
      column = 4,
      row = 3,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command disableCommand =
      Commands.runOnce(this::disable, this).ignoringDisable(true).withName("Disable");

  private DoubleLogEntry logGoalVelocity = new DoubleLogEntry(LOG, "/Shooter/Goal Velocity");
  private DoubleLogEntry logCurrentVelocity = new DoubleLogEntry(LOG, "/Shooter/Current Velocity");

  /** Creates a new Shooter subsystem. */
  public Shooter() {}

  /** Interpolates correct shooter velocity for a given distance from our Hub. */
  public double getPowerFromInterpolationTable(double distance) {
    return SHOOTER_VELOCITIES.get(distance);
  }

  public void setGoalVelocity(double goalVelocity) {
    this.goalVelocity = goalVelocity;
    logGoalVelocity.append(goalVelocity);
  }

  /**
   * Adds or subtracts velocity from upper goal. Mainly for controllers and for experiments in
   * ShootingCommands.java.
   */
  public void addGoalVelocity(double goalVelocityDelta) {
    this.goalVelocity += goalVelocityDelta;
  }

  @Override
  public void disable() {
    goalVelocity = 0;
    logGoalVelocity.append(0);
    leftUpperMotor.stopMotor();
    leftLowerMotor.stopMotor();
    rightLowerMotor.stopMotor();
    rightUpperMotor.stopMotor();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    leftUpperMotor.setIdleMode(idleMode);
    leftLowerMotor.setIdleMode(idleMode);
    rightLowerMotor.setIdleMode(idleMode);
    rightUpperMotor.setIdleMode(idleMode);
  }

  @Override
  public void periodic() {
    updateTelemetry();
    if (goalVelocity != 0) {
      double feedforward = this.feedforward.calculate(goalVelocity);
      double feedback = pidController.calculate(currentVelocity, goalVelocity);
      double motorVoltage = feedforward + feedback;
      leftUpperMotor.setVoltage(motorVoltage);

    } else {
      leftUpperMotor.setVoltage(0);
    }
  }

  private void updateTelemetry() {
    currentVelocity = encoder.getVelocity();
    logCurrentVelocity.append(currentVelocity);
    leftUpperMotor.logTelemetry();
    leftLowerMotor.logTelemetry();
    rightLowerMotor.logTelemetry();
    rightUpperMotor.logTelemetry();
  }
}
