/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Subsystems;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberCommands {

  /** Returns a command that goes to the given elevator level. */
  public static Command seekToElevatorLevel(Subsystems subsystems, ElevatorLevel level) {
    Climber climber = subsystems.climber;

    return Commands.runOnce(() -> climber.setGoalHeight(level), climber)
        .withName(String.format("GoToElevatorLevel(%s)", level.name()));
  }

  /** Returns a command that waits for elevator to reach goal position. */
  public static Command waitForElevatorToReachGoalHeight(Climber climber) {
    return Commands.idle(climber)
        .until(climber::atGoalHeight)
        .withName("WaitForElevatorToReachGoalPosition");
  }

  /** Returns a command that stows elevator. */
  public static Command stowElevator(Subsystems subsystems) {
    return stowElevator(subsystems.climber);
  }

  /**
   * Returns a command that stows elevator.
   *
   * @param climber The climber subsystem.
   * @return A command that stows elevator.
   */
  public static Command stowElevator(Climber climber) {
    return Commands.sequence(
            Commands.runOnce(() -> climber.setGoalHeight(ElevatorLevel.STOWED), climber),
            waitForElevatorToReachGoalHeight(climber),
            Commands.runOnce(() -> climber.disable(), climber))
        .withName("Stow Elevator");
  }
}
