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
import com.nrg948.preferences.DoublePreference;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
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
import frc.robot.util.SimpleMovingAverage;
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
              RobotSelector.AlphaRobot2026, MotorParameters.NullMotor,
              RobotSelector.CompetitionRobot2026, MotorParameters.KrakenX44,
              RobotSelector.PracticeRobot2026, MotorParameters.KrakenX44),
          MotorParameters.NullMotor);

  private static final double GEAR_RATIO = 1.0;
  private static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
  private static final double METERS_PER_REV = (WHEEL_DIAMETER * Math.PI) / GEAR_RATIO;
  private static final double RPS_PER_MPS = 1.0 / METERS_PER_REV;

  public static final double SHOOTER_FEED_VELOCITY = 30;

  @DashboardTextDisplay(title = "Max Velocity (m/s)", column = 0, row = 3, width = 2, height = 1)
  private static final double MAX_VELOCITY =
      (SHOOTER_MOTOR.getFreeSpeedRPM() * METERS_PER_REV / 60.0) * EFFICIENCY;

  public static final InterpolatingDoubleTreeMap SHOOTER_VELOCITIES =
      new InterpolatingDoubleTreeMap();

  static {
    if (isCompBot()) {
      SHOOTER_VELOCITIES.put(1.28, 13.0);
      SHOOTER_VELOCITIES.put(1.35, 13.25);
      SHOOTER_VELOCITIES.put(1.67, 14.0);
      SHOOTER_VELOCITIES.put(2.0, 15.25);
      SHOOTER_VELOCITIES.put(2.33, 16.25);
      SHOOTER_VELOCITIES.put(2.66, 17.5);
      SHOOTER_VELOCITIES.put(3.05, 19.0);
      SHOOTER_VELOCITIES.put(3.35, 21.5);
      SHOOTER_VELOCITIES.put(3.67, 29.5);
    } else {
      SHOOTER_VELOCITIES.put(1.28, 13.25);
      SHOOTER_VELOCITIES.put(1.35, 13.5);
      SHOOTER_VELOCITIES.put(1.67, 14.25);
      SHOOTER_VELOCITIES.put(2.0, 15.5);
      SHOOTER_VELOCITIES.put(2.33, 16.75);
      SHOOTER_VELOCITIES.put(2.66, 18.0);
      SHOOTER_VELOCITIES.put(3.05, 19.5);
      SHOOTER_VELOCITIES.put(3.35, 22.0);
      SHOOTER_VELOCITIES.put(3.67, 30.0);
    }
  }

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
          SHOOTER_MOTOR.newController(
              "/Shooter/Left Lower Motor",
              SHOOTER_LOWER_LEFT_ID,
              CLOCKWISE_POSITIVE,
              COAST,
              METERS_PER_REV);
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
          SHOOTER_MOTOR.newController(
              "/Shooter/Right Lower Motor",
              SHOOTER_LOWER_RIGHT_ID,
              COUNTER_CLOCKWISE_POSITIVE,
              COAST,
              METERS_PER_REV);

  private final RelativeEncoder encoder = leftUpperMotor.getEncoder();

  private final MotionMagicVelocityVoltage flywheelVelocitySmooth =
      new MotionMagicVelocityVoltage(0).withEnableFOC(false);

  private static final int VELOCITY_SMOOTHING_WINDOW = 6;
  private final SimpleMovingAverage velocityFilter =
      new SimpleMovingAverage(VELOCITY_SMOOTHING_WINDOW);

  private static final DoublePreference SHOT_DETECTION_THRESHOLD_MPS =
      new DoublePreference("Shooter", "Shot Detection Threshold MPS", 0.65);

  private final Debouncer shotDebouncer = new Debouncer(0.06, DebounceType.kRising);
  private final Debouncer hopperEmptyDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private int fuelShotCount = 0;

  private boolean lastShotDetected = false;
  private boolean hopperEmpty = false;
  private boolean hasFiredSinceArmed = false;

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

  public double getCurrentVelocity() {
    return currentVelocity;
  }

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
  private DoubleLogEntry logSmoothedVelocity =
      new DoubleLogEntry(LOG, "/Shooter/Smoothed Velocity");
  private DoubleLogEntry logFuelShotCount = new DoubleLogEntry(LOG, "/Shooter/Fuel Shot Count");
  private BooleanLogEntry logHopperEmpty = new BooleanLogEntry(LOG, "/Shooter/Hopper Empty");

  public static final double TOWER_SHOT_DISTANCE = 3.05;
  public static final double HUB_SHOT_DISTANCE = 1.3;
  public static final double MAX_SHOOTING_DISTANCE = 3.7; // TODO: Update for hood angle
  public static final double SHOOTING_RANGE = MAX_SHOOTING_DISTANCE - HUB_SHOT_DISTANCE;

  /** Creates a new Shooter. */
  public Shooter() {
    configureMotionMagic();
    configureFollowers();
  }

  private void configureMotionMagic() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Test new PID values: kP = 0.50, kS = 0.25, kV = 1.0 / 8.35
    double kS = SHOOTER_MOTOR.getKs();
    double kV = (MAX_BATTERY_VOLTAGE - kS) / MAX_VELOCITY;

    config.Slot0.kP = 1.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV * METERS_PER_REV;
    config.Slot0.kA = 0.0;

    // Motion Magic is used only when we adjust to idle speed
    double idleRPM = MAX_VELOCITY * RPS_PER_MPS * 60.0;
    double slowRampTime = 1.5; // seconds to go from 0 to idle speed when "slow" mode is enabled
    config.MotionMagic.MotionMagicAcceleration = idleRPM / 60.0 / slowRampTime;
    config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration / 0.5;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.Voltage.PeakForwardVoltage = MAX_BATTERY_VOLTAGE;
    config.Voltage.PeakReverseVoltage = -MAX_BATTERY_VOLTAGE;

    leftUpperMotor.applyConfiguration(config);
    // -- If using 2 Leaders (one per side) --
    // rightUpperMotor.applyConfiguration(config);
  }

  private void configureFollowers() {
    int leaderID = leftUpperMotor.getDeviceID();
    leftLowerMotor.setFollower(leaderID, false);
    rightUpperMotor.setFollower(leaderID, true);
    rightLowerMotor.setFollower(leaderID, true);

    // -- If using 2 Leaders (one per side) --
    // leftLowerMotor.setFollower(leftUpperMotor.getDeviceID(), false);
    // rightLowerMotor.setFollower(rightUpperMotor.getDeviceID(), false);
  }

  /** Sets shooter goal velocity based on distance inputted to interpolation table. */
  public void setGoalDistance(double distance) {
    setGoalVelocity(SHOOTER_VELOCITIES.get(distance));
    logGoalDistance.append(distance);
  }

  public void setGoalVelocity(double goalVelocity) {
    this.goalVelocity = goalVelocity;
    logGoalVelocity.append(goalVelocity);
  }

  public void addGoalVelocity(double goalVelocityDelta) {
    this.goalVelocity += goalVelocityDelta;
  }

  public boolean atOrNearGoal() {
    return goalVelocity != 0
        && Math.abs(currentVelocity - goalVelocity) / goalVelocity < VELOCITY_PERCENT_TOLERANCE;
  }

  public double getVelocityFromInterpolationTable(double distance) {
    return SHOOTER_VELOCITIES.get(distance);
  }

  private boolean detectFlywheelDrop(double dropMPS) {
    double desiredLinearVelocity = this.goalVelocity;
    double currentLinearVelocity = this.currentVelocity;
    return (currentLinearVelocity - desiredLinearVelocity) <= -dropMPS
        && currentLinearVelocity > 1.0
        && atOrNearGoal();
  }

  public boolean isHopperEmpty() {
    return hopperEmpty;
  }

  public int getFuelShotCount() {
    return fuelShotCount;
  }

  public void armShotDetection() {
    hasFiredSinceArmed = false;
    hopperEmpty = false;
    fuelShotCount = 0;
    hopperEmptyDebouncer.calculate(false);
  }

  public void resetFuelShotCount() {
    fuelShotCount = 0;
    hopperEmpty = false;
  }

  @Override
  public void disable() {
    goalVelocity = 0;
    logGoalVelocity.append(0);
    leftUpperMotor.stopMotor();
    // -- If using 2 Leaders (one per side) --
    // rightUpperMotor.stopMotor();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
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

    if (goalVelocity != 0) {
      double goalRPS = goalVelocity * RPS_PER_MPS;
      leftUpperMotor.setControl(flywheelVelocitySmooth.withVelocity(goalRPS));
      // -- If using 2 Leaders (one per side) --
      // rightUpperMotor.setControl(flywheelVelocitySmooth.withVelocity(goalRPS));
    } else {
      leftUpperMotor.stopMotor();
      // -- If using 2 Leaders (one per side) --
      // rightUpperMotor.stopMotor();
    }

    boolean shotDetected =
        shotDebouncer.calculate(detectFlywheelDrop(SHOT_DETECTION_THRESHOLD_MPS.getValue()));
    if (shotDetected && !lastShotDetected) {
      fuelShotCount++;
      hasFiredSinceArmed = true;
      logFuelShotCount.append(fuelShotCount);
    }
    lastShotDetected = shotDetected;

    hopperEmpty =
        hasFiredSinceArmed && hopperEmptyDebouncer.calculate(atOrNearGoal() && !shotDetected);
    logHopperEmpty.append(hopperEmpty);
  }

  private void updateTelemetry() {
    double rawVelocity = encoder.getVelocity();
    velocityFilter.add(rawVelocity);
    currentVelocity = velocityFilter.getAverage();
    logCurrentVelocity.append(rawVelocity);
    logSmoothedVelocity.append(currentVelocity);
  }
}
