/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorDirection;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.TalonFXAdapter;

public class Elevator extends SubsystemBase implements ActiveSubsystem {

  private final MotorDirection motorDirection = CLOCKWISE_POSITIVE;

  // TODO: Change placeholder values
  private final TalonFXAdapter mainMotor =
      new TalonFXAdapter(
          "/Elevator", new TalonFX(1, "rio"), motorDirection, MotorIdleMode.BRAKE, 0);

  // Trapezoid profile values

  //TODO: replace default values
  /*private static final DCMotor MOTOR_PARAMS = DCMotor.getKrakenX60(1);
  private static final double MAX_SPEED =
      (MOTOR_PARAMS.freeSpeedRadPerSec / (2 * Math.PI)) * METERS_PER_REVOLUTION; // m/s
  private static final double MAX_ACCELERATION =
      (MOTOR_PARAMS.stallTorqueNewtonMeters * GEAR_RATIO) / (SPROCKET_DIAMETER * MASS); // m/s^2
  private static final ExponentialProfile.Constraints EXPONENTIAL_CONSTRAINTS =
      ExponentialProfile.Constraints.fromCharacteristics(
          MAX_BATTERY_VOLTAGE, MAX_SPEED / 2, MAX_ACCELERATION / 64);*/

  // TODO: Change Feedforward constants
  private static final double KS = 0;
  private static final double KV = 0;
  private static final double KA = 0;
  private static final double KG = 9.81 * KA;

  private final ElevatorFeedforward feedForward = new ElevatorFeedforward(KS, KG, KV, KA);

  /** Creates a new Elevator. */
  public Elevator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setIdleMode(MotorIdleMode idleMode) {
    // Do not ever put the elevator in COAST mode or else it will crash down.
  }

  public void disable() {}
}
