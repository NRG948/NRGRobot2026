/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.CANID.SHOOTER_LOWER_2_ID;
import static frc.robot.Constants.RobotConstants.CANID.SHOOTER_UPPER_2_ID;
import static frc.robot.Constants.RobotConstants.MAX_BATTERY_VOLTAGE;
import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorDirection.COUNTER_CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.COAST;

import com.nrg948.dashboard.annotations.DashboardCommand;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.nrg948.dashboard.annotations.DashboardPIDController;
import com.nrg948.dashboard.annotations.DashboardRadialGauge;
import com.nrg948.dashboard.annotations.DashboardTextDisplay;
import com.nrg948.dashboard.model.DataBinding;
import com.nrg948.preferences.PIDControllerPreference;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

  private static final MotorParameters LOWER_MOTOR = MotorParameters.KrakenX44;
  private static final double LOWER_GEAR_RATIO = 9.0; // TODO Finalize gear ratio
  private static final double LOWER_WHEEL_DIAMETER =
      Units.inchesToMeters(4); // TODO Finalize diameter
  private static final double LOWER_METERS_PER_REV =
      (LOWER_WHEEL_DIAMETER * Math.PI) / LOWER_GEAR_RATIO;

  private static final MotorParameters UPPER_MOTOR = MotorParameters.KrakenX44;
  private static final double UPPER_GEAR_RATIO = 9.0; // TODO Finalize gear ratio
  private static final double UPPER_WHEEL_DIAMETER =
      Units.metersToInches(3); // TODO Finalize diameter
  private static final double UPPER_METERS_PER_REV =
      (UPPER_WHEEL_DIAMETER * Math.PI) / UPPER_GEAR_RATIO;

  private final MotorController lowerMotor =
      LOWER_MOTOR.newController(
          "/Shooter/Lower Motor",
          CANID.SHOOTER_LOWER_1_ID,
          CLOCKWISE_POSITIVE,
          COAST,
          LOWER_METERS_PER_REV);
  private final MotorController lowerMotor2 = lowerMotor.createFollower(SHOOTER_LOWER_2_ID, true);
  private final MotorController upperMotor =
      UPPER_MOTOR.newController(
          "/Shooter/Upper Motor",
          CANID.SHOOTER_UPPER_1_ID,
          COUNTER_CLOCKWISE_POSITIVE,
          COAST,
          UPPER_METERS_PER_REV);
  private final MotorController upperMotor2 = upperMotor.createFollower(SHOOTER_UPPER_2_ID, true);

  private static final double LOWER_MAX_VELOCITY =
      LOWER_MOTOR.getFreeSpeedRPM() * LOWER_METERS_PER_REV / 60.0;
  private static final double UPPER_MAX_VELOCITY =
      UPPER_MOTOR.getFreeSpeedRPM() * UPPER_METERS_PER_REV / 60.0;

  private static final double LOWER_KS = LOWER_MOTOR.getKs();
  private static final double UPPER_KS = UPPER_MOTOR.getKs();
  private static final double LOWER_KV = (MAX_BATTERY_VOLTAGE - LOWER_KS) / LOWER_MAX_VELOCITY;
  private static final double UPPER_KV = (MAX_BATTERY_VOLTAGE - UPPER_KS) / UPPER_MAX_VELOCITY;

  private final RelativeEncoder lowerEncoder = lowerMotor.getEncoder();
  private final RelativeEncoder upperEncoder = upperMotor.getEncoder();

  private final SimpleMotorFeedforward lowerFeedforward =
      new SimpleMotorFeedforward(LOWER_KS, LOWER_KV);
  private final SimpleMotorFeedforward upperFeedforward =
      new SimpleMotorFeedforward(UPPER_KS, UPPER_KV);

  @DashboardPIDController(title = "Lower PID", column = 6, row = 0, width = 2, height = 3)
  private final PIDControllerPreference lowerPIDController =
      new PIDControllerPreference("Shooter", "Lower PID Controller", 1, 0, 0);

  @DashboardPIDController(title = "Upper PID", column = 8, row = 0, width = 2, height = 3)
  private final PIDControllerPreference upperPIDController =
      new PIDControllerPreference("Shooter", "Upper PID Controller", 1, 0, 0);

  @DashboardTextDisplay(
      title = "Lower Goal Velocity (m/s)",
      column = 0,
      row = 2,
      width = 2,
      height = 1)
  private double lowerGoalVelocity = 0;

  @DashboardRadialGauge(
      title = "Lower Velocity (m/s)",
      column = 0,
      row = 0,
      width = 2,
      height = 2,
      min = -5.9108,
      max = 5.9108)
  private double lowerCurrentVelocity = 0; // Different Velocity for upper and lower pairs of motors

  @DashboardTextDisplay(
      title = "Upper Goal Velocity (m/s)",
      column = 2,
      row = 2,
      width = 2,
      height = 1)
  private double upperGoalVelocity = 0;

  @DashboardRadialGauge(
      title = "Upper Velocity (m/s)",
      column = 2,
      row = 0,
      width = 2,
      height = 2,
      min = -4.4331,
      max = 4.4331)
  private double upperCurrentVelocity = 0;

  @DashboardTextDisplay(
      title = "Test Lower Velocity (m/s)",
      column = 4,
      row = 0,
      width = 2,
      height = 1,
      dataBinding = DataBinding.READ_WRITE,
      showSubmitButton = true)
  private double testLowerGoalVelocity = 0;

  @DashboardTextDisplay(
      title = "Test Upper Velocity (m/s)",
      column = 4,
      row = 1,
      width = 2,
      height = 1,
      dataBinding = DataBinding.READ_WRITE,
      showSubmitButton = true)
  private double testUpperGoalVelocity = 0;

  @DashboardCommand(
      title = "Set Test Velocities (m/s)",
      column = 4,
      row = 2,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command setTestGoalVelocitiesCommand =
      Commands.runOnce(() -> setGoalVelocities(testLowerGoalVelocity, testUpperGoalVelocity), this)
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

  private DoubleLogEntry logLowerGoalVelocity =
      new DoubleLogEntry(LOG, "/Shooter/Lower Goal Velocity");
  private DoubleLogEntry logUpperGoalVelocity =
      new DoubleLogEntry(LOG, "/Shooter/Upper Goal Velocity");
  private DoubleLogEntry logLowerCurrentVelocity =
      new DoubleLogEntry(LOG, "/Shooter/Lower Current Velocity");
  private DoubleLogEntry logUpperCurrentVelocity =
      new DoubleLogEntry(LOG, "/Shooter/Upper Current Velocity");

  /** Creates a new Shooter. */
  public Shooter() {}

  public void setGoalVelocities(double lowerGoalVelocity, double upperGoalVelocity) {
    this.lowerGoalVelocity = lowerGoalVelocity;
    this.upperGoalVelocity = upperGoalVelocity;
    logLowerGoalVelocity.append(lowerGoalVelocity);
    logUpperGoalVelocity.append(upperGoalVelocity);
  }

  @Override
  public void disable() {
    lowerGoalVelocity = 0;
    upperGoalVelocity = 0;
    logLowerGoalVelocity.append(0);
    logUpperGoalVelocity.append(0);
    lowerMotor.stopMotor();
    lowerMotor2.stopMotor();
    upperMotor.stopMotor();
    upperMotor2.stopMotor();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    lowerMotor.setIdleMode(idleMode);
    lowerMotor2.setIdleMode(idleMode);
    upperMotor.setIdleMode(idleMode);
    upperMotor2.setIdleMode(idleMode);
  }

  @Override
  public void periodic() {
    updateTelemetry();
    if (lowerGoalVelocity != 0) {
      double feedforward = this.lowerFeedforward.calculate(lowerGoalVelocity);
      double feedback = lowerPIDController.calculate(lowerCurrentVelocity, lowerGoalVelocity);
      double motorVoltage = feedforward + feedback;
      lowerMotor.setVoltage(motorVoltage);

    } else {
      lowerMotor.setVoltage(0);
    }
    if (upperGoalVelocity != 0) {
      double feedforward = this.upperFeedforward.calculate(upperGoalVelocity);
      double feedback = upperPIDController.calculate(upperCurrentVelocity, upperGoalVelocity);
      double motorVoltage = feedforward + feedback;
      upperMotor.setVoltage(motorVoltage);

    } else {
      upperMotor.setVoltage(0);
    }
  }

  private void updateTelemetry() {
    lowerCurrentVelocity = lowerEncoder.getVelocity();
    upperCurrentVelocity = upperEncoder.getVelocity();
    logLowerCurrentVelocity.append(lowerCurrentVelocity);
    logUpperCurrentVelocity.append(upperCurrentVelocity);
    lowerMotor.logTelemetry();
    lowerMotor2.logTelemetry();
    upperMotor.logTelemetry();
    upperMotor2.logTelemetry();
  }
}
