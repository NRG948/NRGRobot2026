/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.LEDSegment.STATUS_FIRST_LED;
import static frc.robot.Constants.RobotConstants.LEDSegment.STATUS_LED_COUNT;
import static frc.robot.parameters.Colors.RED;
import static frc.robot.util.MatchUtil.transitionTimes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.LEDs.BlinkColor;
import frc.robot.commands.LEDs.BlinkingRainbowCycle;
import frc.robot.commands.LEDs.LEDCommands;
import frc.robot.commands.LEDs.RainbowCycle;
import frc.robot.parameters.Colors;
import frc.robot.util.MatchUtil;
import frc.robot.util.MatchUtil.LEDLights;

/** A subsystem to control the status LEDs. */
public final class StatusLED extends LEDSubsystem {

  private LEDLights lastLights = LEDLights.NONE;

  /** Creates a new StatusLEDSubsystem. */
  public StatusLED() {
    super(STATUS_FIRST_LED, STATUS_LED_COUNT);
    leds.fill(RED);
    leds.commitColor();
  }

  @Override
  public void periodic() {
    /** Adds one because the fractional seconds gets truncated by the API.  */
    int timeRemaining = (int) MatchUtil.getMatchTimeRemaining() + 1;
    LEDLights light = transitionTimes.get((Integer) timeRemaining);

    if (light != null && lastLights != light && MatchUtil.isTeleop()) {
      lastLights = light;
      Command toSchedule =
          switch (light) {
            case GREEN:
              yield LEDCommands.setColorAndIdle(this, Colors.GREEN);
            case BLINKING_YELLOW:
              yield new BlinkColor(this, Colors.YELLOW);
            case BLINKING_RED:
              yield new BlinkColor(this, Colors.RED);
            case RAINBOW:
              yield new RainbowCycle(this);
            case BLINKING_RAINBOW:
              yield new BlinkingRainbowCycle(this);
            case NONE:
              yield Commands.none();
          };
      CommandScheduler.getInstance().schedule(toSchedule);
    }
  }
}
