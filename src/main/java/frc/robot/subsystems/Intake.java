/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.BRAKE;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorDirection;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;
import frc.robot.util.TalonFXAdapter;

public class Intake extends SubsystemBase implements ActiveSubsystem {

  private final MotorDirection motorDirection = CLOCKWISE_POSITIVE;

  private final TalonFXAdapter pivotMotor =
      new TalonFXAdapter("/pivotMotor", new TalonFX(0, "rio"), motorDirection, BRAKE, 0);
  private final TalonFXAdapter rollerMotor =
      new TalonFXAdapter("/rollerMotor", new TalonFX(0, "rio"), motorDirection, BRAKE, 0);

  private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
  private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();

  private final double KS = 0;
  private final double KV = 0;

  private final SimpleMotorFeedforward pivotFeedforward = new SimpleMotorFeedforward(KS, KV);
  private final SimpleMotorFeedforward rollerFeedforward = new SimpleMotorFeedforward(KS, KV);
  private final PIDController pivotPIDController = new PIDController(1, 0, 0);
  private final PIDController rollerPIDController = new PIDController(1, 0, 0);

  private double goalPivotVelocity = 0;
  private double currentPivotVelocity = 0;
  private double pivotVoltage = 0;
  private double goalRollerVelocity = 0;
  private double currentRollerVelocity = 0;
  private double rollerVoltage = 0;

  /** Creates a new Intake. */
  public Intake() {}

  public void setPivotVelocity(double goalPivotVelocity) {
    this.goalPivotVelocity = goalPivotVelocity;
  }

  public void setRollerVelocity(double goalRollerVelocity) {
    this.goalRollerVelocity = goalRollerVelocity;
  }

  @Override
  public void disable() {
    goalPivotVelocity = 0;
    goalRollerVelocity = 0;
    pivotMotor.stopMotor();
    rollerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    updateTelemetry();

    if (goalPivotVelocity != 0) {
      double feedforward = pivotFeedforward.calculate(currentPivotVelocity);
      double feedback = pivotPIDController.calculate(currentPivotVelocity, goalPivotVelocity);
      double pivotVoltage = feedforward + feedback;
      pivotMotor.setVoltage(pivotVoltage);

    } else {
      pivotMotor.setVoltage(0);
    }

    if (goalRollerVelocity != 0) {
      double feedforward = rollerFeedforward.calculate(currentRollerVelocity);
      double feedback = rollerPIDController.calculate(currentRollerVelocity, goalRollerVelocity);
      double rollerVoltage = feedforward + feedback;
      rollerMotor.setVoltage(rollerVoltage);

    } else {
      rollerMotor.setVoltage(0);
    }
  }

  private void updateTelemetry() {
    currentPivotVelocity = pivotEncoder.getVelocity();
    currentRollerVelocity = rollerEncoder.getVelocity();
    pivotMotor.logTelemetry();
    rollerMotor.logTelemetry();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    pivotMotor.setIdleMode(idleMode);
    rollerMotor.setIdleMode(idleMode);
  }
}
