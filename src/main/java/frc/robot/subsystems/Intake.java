// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorDirection;
import frc.robot.util.TalonFXAdapter;

public class Intake extends SubsystemBase {
  private final MotorDirection motorDirection = CLOCKWISE_POSITIVE;
  private final TalonFXAdapter pivotMotor = new TalonFXAdapter(getName(), new TalonFX(0, "rio"), motorDirection, 0);
  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}   
