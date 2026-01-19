/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CANID;
import frc.robot.parameters.MotorParameters;
import frc.robot.util.MotorController;
import frc.robot.util.MotorDirection;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;

public class Indexer extends SubsystemBase implements ActiveSubsystem {

  private static final MotorParameters MOTOR = MotorParameters.KrakenX60;

  private final MotorController indexerMotor =
      MOTOR.newController(
          "/indexerMotor",
          CANID.INDEXER_ID,
          MotorDirection.CLOCKWISE_POSITIVE,
          MotorIdleMode.BRAKE,
          0);
  private final RelativeEncoder indexEncoder = indexerMotor.getEncoder();

  private final double KS = 0;
  private final double KV = 0;
  private final PIDController pidController = new PIDController(1, 0, 0);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV);

  private double goalVelocity = 0;
  private double currentVelocity = 0;

  /** Creates a new Indexer. */
  public Indexer() {}

  public void setVelocity(double goalVelocity) {
    this.goalVelocity = goalVelocity;
  }

  @Override
  public void disable() {
    goalVelocity = 0;
    indexerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    updateTelemetry();

    if (goalVelocity != 0) {
      double feedforward = this.feedforward.calculate(goalVelocity);
      double feedback = pidController.calculate(currentVelocity, goalVelocity);
      double voltage = feedforward + feedback;
      indexerMotor.setVoltage(voltage);

    } else {
      indexerMotor.setVoltage(0);
    }
  }

  private void updateTelemetry() {
    currentVelocity = indexEncoder.getVelocity();
    indexerMotor.logTelemetry();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    indexerMotor.setIdleMode(idleMode);
  }
}
