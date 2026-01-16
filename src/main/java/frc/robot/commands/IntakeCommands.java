/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;

/** Add your docs here. */
public class IntakeCommands {

  public Command stowIntake(Intake intake) {
    return Commands.none();
  }

  public Command setIntakeSpeed(double intakeSpeed, Intake intake) {
    return Commands.runOnce(() -> intake.setRollerVelocity(intakeSpeed), intake);
  }

  public Command setIntakeAngle(double intakeAngle, Intake intake) {
    return Commands.none();
  }
}
