/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.CANID.INTAKE_ARM_ID;
import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.BRAKE;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.parameters.MotorParameters;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;
import frc.robot.util.TalonFXAdapter;

@DashboardDefinition
public class IntakeArm extends SubsystemBase implements ActiveSubsystem {

  private final double TOLERANCE = 0; // TODO: Add tolerance in radians
  private final double RADIANS_PER_ROTATIONS = 1; // TODO: Calculate radians per rotations
  private final double ERROR_MARGIN = 0; // TODO: Add error margin in radians
  private final double ERROR_TIME = 1;
  private final double GEAR_RATIO = 0; // TODO: Find gear ratio

  // TODO: Find intake arm angles
  private final double stowAngle = 0;
  private final double minAngle = 0;
  private final double maxAngle = 0;

  private static final MotorParameters MOTOR = MotorParameters.KrakenX60;

  private final TalonFXAdapter motor =
      new TalonFXAdapter(
          "/IntakeArm/Motor",
          new TalonFX(INTAKE_ARM_ID),
          CLOCKWISE_POSITIVE,
          BRAKE,
          RADIANS_PER_ROTATIONS);

  private final RelativeEncoder encoder = motor.getEncoder();

  private double currentAngle = 0;
  private double goalAngle = 0;
  private double currentVelocity = 0;
  private boolean enabled;
  private boolean hasError = false;

  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private final Timer stuckTimer = new Timer();

  /** Creates a new IntakeArm. */
  public IntakeArm() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    FeedbackConfigs feedbackConfigs = talonFXConfigs.Feedback;
    feedbackConfigs.SensorToMechanismRatio = GEAR_RATIO;

    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0;
    slot0Configs.kV = 0;
    slot0Configs.kA = 0;
    slot0Configs.kG = 0.9;
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Configs.kP = 120;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0;
    motionMagicConfigs.MotionMagicAcceleration = 0;

    TalonFXConfigurator configurator =
        new TalonFXConfigurator(new DeviceIdentifier(INTAKE_ARM_ID, "KrakenX60", CANBus.roboRIO()));
    configurator.apply(slot0Configs);
  }

  public void updateTelemetry() {
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
        currentAngle > maxAngle + ERROR_MARGIN
            || currentAngle < minAngle - ERROR_MARGIN
            || stuckTimer.hasElapsed(ERROR_TIME);
  }

  /**
   * Sets motor periodic control to desired goal angle
   *
   * @param angle angle in radians
   */
  public void setGoalAngle(double angle) {
    angle = MathUtil.clamp(angle, minAngle, maxAngle);
    goalAngle = angle;
    enabled = true;

    motor.setControl(motionMagicRequest.withPosition(angle / RADIANS_PER_ROTATIONS).withSlot(0));
  }

  /**
   * Returns whether the intake arm is near goal angle
   *
   * @param goalAngle
   * @return
   */
  public boolean atGoalAngle(double goalAngle) {
    return Math.abs(goalAngle - currentAngle) <= TOLERANCE;
  }

  public boolean atGoalAngle() {
    return atGoalAngle(this.goalAngle);
  }

  public boolean isStowed() {
    return atGoalAngle(this.stowAngle);
  }

  public boolean hasError() {
    return hasError;
  }

  public double getStowAngle() {
    return stowAngle;
  }

  @Override
  public void disable() {
    enabled = false;
    motor.disable();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateTelemetry();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    motor.setIdleMode(idleMode);
  }
}
