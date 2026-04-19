/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.parameters.Colors;
import frc.robot.subsystems.StatusLED;
import frc.robot.subsystems.Subsystems;

/** A namespace for LED command factory methods. */
public final class LEDCommands {

  /**
   * Returns a command that sets the color of the status LEDs.
   *
   * @param statusLED The status LED subsystem.
   * @param color The color to set.
   * @return A command that sets the color of the status LED.
   */
  public static Command setColorAndIdle(StatusLED statusLED, Colors color) {
    return Commands.sequence(
            Commands.runOnce(() -> statusLED.fillAndCommitColor(color), statusLED),
            Commands.idle(statusLED))
        .withName(String.format("SetColorAndIdle(%s)", color.name()));
  }

  /*
   * Returns a command that sets the autonomous LEDs to Flame Cycle.
   *
   */
  public static Command autoLEDs(Subsystems subsystems) {
    return subsystems
        .statusLEDs
        .map(statusLEDS -> (Command) new FlameCycle(statusLEDS))
        .orElse(Commands.none());
  }
}
