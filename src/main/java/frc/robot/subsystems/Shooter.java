/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.CANID.SHOOTER_LOWER_LEFT_ID;
import static frc.robot.Constants.RobotConstants.CANID.SHOOTER_LOWER_RIGHT_ID;
import static frc.robot.Constants.RobotConstants.CANID.SHOOTER_UPPER_LEFT_ID;
import static frc.robot.Constants.RobotConstants.CANID.SHOOTER_UPPER_RIGHT_ID;
import static frc.robot.Constants.RobotConstants.MAX_BATTERY_VOLTAGE;
import static frc.robot.RobotPreferences.isCompBot;
import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorDirection.COUNTER_CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.COAST;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.nrg948.dashboard.annotations.DashboardCommand;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.nrg948.dashboard.annotations.DashboardRadialGauge;
import com.nrg948.dashboard.annotations.DashboardTextDisplay;
import com.nrg948.dashboard.model.DataBinding;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotPreferences;
import frc.robot.RobotSelector;
import frc.robot.parameters.MotorParameters;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;
import frc.robot.util.TalonFXAdapter;
import java.util.Map;

@DashboardDefinition
public final class Shooter extends SubsystemBase implements ActiveSubsystem {
  private static final double VELOCITY_PERCENT_TOLERANCE = 0.03;
  private static final double EFFICIENCY = 0.9;

  private static final DataLog LOG = DataLogManager.getLog();

  private static final MotorParameters SHOOTER_MOTOR =
      RobotPreferences.ROBOT_TYPE.selectOrDefault(
          Map.of(
              RobotSelector.CompetitionRobot2026, MotorParameters.KrakenX44,
              RobotSelector.PracticeRobot2026, MotorParameters.KrakenX44),
          MotorParameters.KrakenX44);

  private static final double GEAR_RATIO = 1.0;
  private static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
  private static final double METERS_PER_REV = (WHEEL_DIAMETER * Math.PI) / GEAR_RATIO;

  public static final double SHOOTER_FEED_VELOCITY = 30;

  @DashboardTextDisplay(title = "Max Velocity (m/s)", column = 0, row = 3, width = 2, height = 1)
  private static final double MAX_VELOCITY =
      (SHOOTER_MOTOR.getFreeSpeedRPM() * METERS_PER_REV / 60.0) * EFFICIENCY;

  private static final InterpolatingDoubleTreeMap SHOOTER_VELOCITIES =
      new InterpolatingDoubleTreeMap();

  static {
    if (isCompBot()) {
      // Competition bot 70 degree hood
      SHOOTER_VELOCITIES.put(1.28, 12.75);
      SHOOTER_VELOCITIES.put(1.35, 13.0);
      SHOOTER_VELOCITIES.put(1.67, 13.75);
      SHOOTER_VELOCITIES.put(2.0, 14.75);
      SHOOTER_VELOCITIES.put(2.33, 15.5);
      SHOOTER_VELOCITIES.put(2.66, 16.5);
      SHOOTER_VELOCITIES.put(3.05, 17.5);
      SHOOTER_VELOCITIES.put(3.35, 20.0);
      SHOOTER_VELOCITIES.put(3.67, 27.75);
    } else {
      // Practice bot 70 degree hood
      SHOOTER_VELOCITIES.put(1.28, 12.75);
      SHOOTER_VELOCITIES.put(1.35, 13.0);
      SHOOTER_VELOCITIES.put(1.67, 13.75);
      SHOOTER_VELOCITIES.put(2.0, 15.0);
      SHOOTER_VELOCITIES.put(2.33, 15.75);
      SHOOTER_VELOCITIES.put(2.66, 17.0);
      SHOOTER_VELOCITIES.put(3.05, 18.25);
      SHOOTER_VELOCITIES.put(3.35, 20.75);
      SHOOTER_VELOCITIES.put(3.67, 28.75);
    }
  }

  // These motors can only be controlled by a TalonFX Motor Controller
  private final TalonFXAdapter leftUpperMotor =
      (TalonFXAdapter)
          SHOOTER_MOTOR.newController(
              "/Shooter/Left Upper Motor",
              SHOOTER_UPPER_LEFT_ID,
              CLOCKWISE_POSITIVE,
              COAST,
              METERS_PER_REV);
  private final TalonFXAdapter leftLowerMotor =
      (TalonFXAdapter)
          leftUpperMotor.createFollower("/Shooter/Left Lower Motor", SHOOTER_LOWER_LEFT_ID, false);
  private final TalonFXAdapter rightUpperMotor =
      (TalonFXAdapter)
          SHOOTER_MOTOR.newController(
              "/Shooter/Right Upper Motor",
              SHOOTER_UPPER_RIGHT_ID,
              COUNTER_CLOCKWISE_POSITIVE,
              COAST,
              METERS_PER_REV);
  private final TalonFXAdapter rightLowerMotor =
      (TalonFXAdapter)
          rightUpperMotor.createFollower(
              "/Shooter/Right Lower Motor", SHOOTER_LOWER_RIGHT_ID, false);

  private final RelativeEncoder encoder = rightUpperMotor.getEncoder();

  private final MotionMagicVelocityVoltage motionMagicVelocityRequest =
      new MotionMagicVelocityVoltage(0).withEnableFOC(false);

  @DashboardTextDisplay(title = "Goal Velocity (m/s)", column = 0, row = 2, width = 2, height = 1)
  private double goalVelocity = 0;

  @DashboardRadialGauge(
      title = "Velocity (m/s)",
      column = 0,
      row = 0,
      width = 2,
      height = 2,
      min = -41.270725699090676,
      max = 41.270725699090676)
  private double currentVelocity = 0;

  @DashboardTextDisplay(
      title = "Test Velocity (m/s)",
      column = 2,
      row = 0,
      width = 2,
      height = 1,
      dataBinding = DataBinding.READ_WRITE,
      showSubmitButton = true)
  private double testGoalVelocity = 0;

  @DashboardCommand(
      title = "Set Test Velocities (m/s)",
      column = 2,
      row = 1,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command setTestGoalVelocitiesCommand =
      Commands.runOnce(() -> setGoalVelocity(testGoalVelocity), this)
          .withName("Set Test Velocities");

  @DashboardCommand(
      title = "Disable",
      column = 2,
      row = 2,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command disableCommand =
      Commands.runOnce(this::disable, this).ignoringDisable(true).withName("Disable");

  private DoubleLogEntry logGoalVelocity = new DoubleLogEntry(LOG, "/Shooter/Goal Velocity");
  private DoubleLogEntry logGoalDistance = new DoubleLogEntry(LOG, "/Shooter/Goal Distance");
  private DoubleLogEntry logCurrentVelocity = new DoubleLogEntry(LOG, "/Shooter/Current Velocity");

  public static final double TOWER_SHOT_DISTANCE = 3.05;
  public static final double HUB_SHOT_DISTANCE = 1.3;
  public static final double MAX_SHOOTING_DISTANCE = 3.7; // TODO: Update for hood angle
  public static final double SHOOTING_RANGE = MAX_SHOOTING_DISTANCE - HUB_SHOT_DISTANCE;
  private static final double SLOW_RAMP_TIME = 0.5;

  /** Creates a new Shooter. */
  public Shooter() {
    configureMotionMagic();
  }

  private void configureMotionMagic() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    double kS = SHOOTER_MOTOR.getKs();
    double kV = (MAX_BATTERY_VOLTAGE - kS) / MAX_VELOCITY;

    config.Slot0.kP = 1.0 * METERS_PER_REV;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV * METERS_PER_REV;
    config.Slot0.kA = 0.0;

    // TODO: Tune acceleration and jerk values for faster response.
    double acceleration = SHOOTER_MOTOR.getFreeSpeedRPM() / 60.0 / SLOW_RAMP_TIME;
    config.MotionMagic.MotionMagicAcceleration = acceleration;
    config.MotionMagic.MotionMagicJerk = acceleration * 2;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.Voltage.PeakForwardVoltage = MAX_BATTERY_VOLTAGE;
    config.Voltage.PeakReverseVoltage = -MAX_BATTERY_VOLTAGE;

    config.MotorOutput = leftUpperMotor.getMotorOutputConfig();
    leftUpperMotor.applyTalonFXConfiguration(config);
    config.MotorOutput = rightUpperMotor.getMotorOutputConfig();
    rightUpperMotor.applyTalonFXConfiguration(config);
  }

  /** Sets shooter goal velocity based on distance inputted to interpolation table. */
  public void setGoalDistance(double distance) {
    setGoalVelocity(SHOOTER_VELOCITIES.get(distance));
    logGoalDistance.append(distance);
  }

  public void setGoalVelocity(double goalVelocity) {
    this.goalVelocity = goalVelocity;
    logGoalVelocity.append(goalVelocity);

    if (goalVelocity != 0) {
      // Convert linear velocity goal to rotational velocity in revolutions per second for Motion
      // Magic.
      double goalRPS = goalVelocity / METERS_PER_REV;
      leftUpperMotor.setControl(motionMagicVelocityRequest.withVelocity(goalRPS));
      rightUpperMotor.setControl(motionMagicVelocityRequest.withVelocity(goalRPS));
    } else {
      leftUpperMotor.stopMotor();
      rightUpperMotor.stopMotor();
    }
  }

  /** Returns whether the shooter velocity has reached its goal. */
  public boolean atOrNearGoal() {
    return (goalVelocity != 0)
        && (Math.abs(currentVelocity - goalVelocity) / goalVelocity) < VELOCITY_PERCENT_TOLERANCE;
  }

  public double getVelocityFromInterpolationTable(double distance) {
    return SHOOTER_VELOCITIES.get(distance);
  }

  @Override
  public void disable() {
    goalVelocity = 0;
    logGoalVelocity.append(0);
    leftUpperMotor.stopMotor();
    rightUpperMotor.stopMotor();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    // Followers do not inherit idle mode configurations when the leader's idle mode changes.
    leftUpperMotor.setIdleMode(idleMode);
    leftLowerMotor.setIdleMode(idleMode);
    rightLowerMotor.setIdleMode(idleMode);
    rightUpperMotor.setIdleMode(idleMode);
  }

  @Override
  public boolean isEnabled() {
    return goalVelocity != 0;
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  private void updateTelemetry() {
    currentVelocity = encoder.getVelocity();
    logCurrentVelocity.append(currentVelocity);
    leftUpperMotor.logTelemetry();
    leftLowerMotor.logTelemetry();
    rightUpperMotor.logTelemetry();
    rightLowerMotor.logTelemetry();
  }
}
