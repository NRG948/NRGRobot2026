/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.CANID.INTAKE_ARM_ID;
import static frc.robot.Constants.RobotConstants.MAX_BATTERY_VOLTAGE;
import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.BRAKE;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.nrg948.dashboard.annotations.DashboardCommand;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.nrg948.dashboard.annotations.DashboardRadialGauge;
import com.nrg948.dashboard.annotations.DashboardTextDisplay;
import com.nrg948.dashboard.model.DataBinding;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
public final class IntakeArm extends SubsystemBase implements ActiveSubsystem {
  private static final MotorParameters MOTOR =
      RobotPreferences.ROBOT_TYPE.selectOrDefault(
          Map.of(
              RobotSelector.AlphaRobot2026, MotorParameters.NullMotor,
              RobotSelector.CompetitionRobot2026, MotorParameters.KrakenX60,
              RobotSelector.PracticeRobot2026, MotorParameters.KrakenX60),
          MotorParameters.NullMotor);

  private static final double TOLERANCE =
      Units.degreesToRadians(5); // TODO: Add tolerance in radians
  private static final double ERROR_MARGIN =
      Units.degreesToRadians(5.0); // TODO: Add error margin in radians
  private static final double ERROR_TIME = 1;

  private static final double GEAR_RATIO = 50.0;
  private static final double RADIANS_PER_ROTATION = 2 * Math.PI;
  private static final double MASS = Units.lbsToKilograms(5.5);
  private static final double LENGTH = Units.inchesToMeters(10.94);
  private static final double MAX_VELOCITY =
      (RADIANS_PER_ROTATION * MOTOR.getFreeSpeedRPM()) / 60.0;
  private static final double MAX_ACCELERATION =
      (MOTOR.getStallTorque() * GEAR_RATIO) / ((MASS * LENGTH * LENGTH) / 3.0);

  public static final double STOW_ANGLE = Units.degreesToRadians(140);
  public static final double BUMP_ANGLE = Units.degreesToRadians(25);
  public static final double EXTENDED_ANGLE = Units.degreesToRadians(0);
  public static final double MIN_ANGLE = Units.degreesToRadians(0);
  public static final double MAX_ANGLE = STOW_ANGLE;

  private final TalonFX talonFX = new TalonFX(INTAKE_ARM_ID);
  private final TalonFXAdapter motor =
      new TalonFXAdapter(
          "/IntakeArm/Motor", talonFX, CLOCKWISE_POSITIVE, BRAKE, RADIANS_PER_ROTATION);

  private final RelativeEncoder encoder = motor.getEncoder();

  private double currentAngle = 0;
  private double goalAngle = 0;
  private double currentVelocity = 0;
  private boolean enabled;
  private boolean hasError = false;

  @DashboardCommand(
      title = "Set Extended Position",
      column = 2,
      row = 0,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command setExtendedPositionCommand =
      Commands.runOnce(() -> encoder.setPosition(EXTENDED_ANGLE), this)
          .withName("Set Extended Position")
          .ignoringDisable(true);

  @DashboardTextDisplay(
      title = "Test Goal Angle",
      column = 2,
      row = 1,
      width = 2,
      height = 1,
      dataBinding = DataBinding.READ_WRITE,
      showSubmitButton = true)
  private double testGoalAngle = 0;

  @DashboardCommand(
      title = "Set Test Goal Angle",
      column = 2,
      row = 2,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command setTestGoalAngleCommand =
      Commands.runOnce(() -> setGoalAngle(Math.toRadians(testGoalAngle)), this)
          .withName("Set Goal Angle");

  @DashboardCommand(
      title = "Disable",
      column = 2,
      row = 3,
      width = 2,
      height = 1,
      fillWidget = true)
  private Command disableCommand =
      Commands.runOnce(this::disable, this).ignoringDisable(true).withName("Disable");

  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private final Timer stuckTimer = new Timer();

  /** Creates a new IntakeArm. */
  public IntakeArm() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    MotorOutputConfigs motorOutputConfigs = talonFXConfigs.MotorOutput;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

    FeedbackConfigs feedbackConfigs = talonFXConfigs.Feedback;
    feedbackConfigs.SensorToMechanismRatio = GEAR_RATIO;

    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = MOTOR.getKs();
    slot0Configs.kV = RADIANS_PER_ROTATION * (MAX_BATTERY_VOLTAGE - MOTOR.getKs()) / MAX_VELOCITY;
    slot0Configs.kA =
        RADIANS_PER_ROTATION * (MAX_BATTERY_VOLTAGE - MOTOR.getKs()) / MAX_ACCELERATION;
    slot0Configs.kG = 0.9;
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Configs.kP = 90;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = MAX_VELOCITY / RADIANS_PER_ROTATION / 200;
    motionMagicConfigs.MotionMagicAcceleration = MAX_ACCELERATION / RADIANS_PER_ROTATION / 30;
    TalonFXConfigurator configurator = talonFX.getConfigurator();
    configurator.apply(talonFXConfigs);

    encoder.setPosition(STOW_ANGLE);
  }

  /** Polls sensors and logs telemetry. */
  private void updateTelemetry() {
    currentAngle = encoder.getPosition();
    currentVelocity = encoder.getVelocity();
    motor.logTelemetry();
  }

  /**
   * Checks if arm is beyond its maximum and minimum angle by 2 degrees Or is taking more than 1
   * second to be within 2 degrees of the goal angle
   */
  private void checkError() {
    if (MathUtil.isNear(goalAngle, currentAngle, TOLERANCE)) {
      stuckTimer.stop();
      stuckTimer.reset();
    } else {
      if (!stuckTimer.isRunning()) {
        stuckTimer.restart();
      }
    }
    hasError =
        currentAngle > MAX_ANGLE + ERROR_MARGIN
            || currentAngle < MIN_ANGLE - ERROR_MARGIN
            || stuckTimer.hasElapsed(ERROR_TIME);
  }

  /**
   * Sets motor periodic control to desired goal angle
   *
   * @param angle angle in radians
   */
  public void setGoalAngle(double angle) {
    angle = MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE);
    goalAngle = angle;
    enabled = true;

    motor.setControl(motionMagicRequest.withPosition(angle / RADIANS_PER_ROTATION).withSlot(0));
  }

  /**
   * {@return the current intake arm angle in degrees} The angle is relative to horizontal with
   * positive values upward.
   */
  @DashboardRadialGauge(
      title = "Current Angle",
      column = 0,
      row = 0,
      width = 2,
      height = 2,
      startAngle = -180,
      endAngle = 180,
      min = -180,
      max = 180,
      numberOfLabels = 0,
      wrapValue = true)
  public double getCurrentAngleDegrees() {
    return Math.toDegrees(currentAngle);
  }

  /**
   * {@return the goal angle in degress} The angle is relative to horizontal with positive values
   * upward.
   */
  @DashboardTextDisplay(
      title = "Goal Angle",
      column = 0,
      row = 2,
      width = 2,
      height = 1,
      dataBinding = DataBinding.READ_ONLY)
  public double getGoalAngleDegrees() {
    return Math.toDegrees(goalAngle);
  }

  /**
   * {@return whether the intake arm is near the specified angle}
   *
   * @param goalAngle The angle to check, in radians.
   */
  private boolean atAngle(double goalAngle) {
    return Math.abs(goalAngle - currentAngle) <= TOLERANCE;
  }

  /** {@return whether the intake arm is near the goal angle} */
  public boolean atGoalAngle() {
    return atAngle(this.goalAngle);
  }

  /** {@return whether the intake arm is near the stowed angle} */
  public boolean isStowed() {
    return atAngle(STOW_ANGLE);
  }

  @Override
  public void disable() {
    enabled = false;
    motor.disable();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    motor.setIdleMode(idleMode);
  }

  @Override
  public boolean isEnabled() {
    return enabled;
  }

  /** {@return the current arm velocity in rads/s} */
  public double getCurrentVelocity() {
    return currentVelocity;
  }

  /** {@return whether an error has been detected} */
  public boolean hasError() {
    return hasError;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateTelemetry();
    checkError();

    if (goalAngle == EXTENDED_ANGLE && currentAngle < EXTENDED_ANGLE) {
      encoder.setPosition(EXTENDED_ANGLE);
    }
    if (goalAngle == STOW_ANGLE && currentAngle > STOW_ANGLE) {
      encoder.setPosition(STOW_ANGLE);
    }
  }
}
