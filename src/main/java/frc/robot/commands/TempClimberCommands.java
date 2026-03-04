// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Subsystems;

//TODO: Delete this class after merging with actual ClimberCommands (once that's pushed)
public class TempClimberCommands extends Command {
  /** Creates a new TempClimberCommands. */
  public TempClimberCommands() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /** Returns a command that goes to the given elevator (climber) level. */
  public static Command seekToElevatorLevel(Subsystems subsystems, ElevatorLevel level) {
    Climber climber = subsystems.climber;
    climber.setIsClimbing(true);


    return Commands.runOnce(() -> climber.setGoalHeight(level), climber)
        .withName(String.format("GoToElevatorLevel(%s)", level.name()));
  }

  /** Returns a command that waits for climber to reach goal position. */
  public static Command waitForClimberToReachGoalHeight(Climber climber) {
    return Commands.idle(climber)
        .until(climber::atGoalHeight)
        .withName("WaitForClimberToReachGoalPosition");
  }
  /** Returns a command that stows climber. */
  public static Command stowClimber(Subsystems subsystems) {
    return stowClimber(subsystems.climber);
  }

  /**
   * Returns a command that stows climber.
   *
   * @param climber The climber subsystem.
   * @return A command that stows climber.
   */
  public static Command stowClimber(Climber climber) {
    return Commands.sequence(
            Commands.runOnce(() -> climber.setGoalHeight(ElevatorLevel.STOWED), climber),
            waitForClimberToReachGoalHeight(climber),
            Commands.runOnce(() -> climber.disable(), climber))
        .withName("Stow Climber");
  }


  /**
   * 
   * @param climber The climber subsystem.
   * @param currLevel The current climb level to unclimb from.
   * @return A command that unclimbs from the current level
   */
  public static Command unClimb(Climber climber, ElevatorLevel currLevel) {
    return Commands.sequence(
            Commands.runOnce(() -> climber.setGoalHeight(currLevel), climber),
            waitForClimberToReachGoalHeight(climber),
            Commands.runOnce(() -> climber.disable(), climber))
        .withName("Unclimb");
  }
}
