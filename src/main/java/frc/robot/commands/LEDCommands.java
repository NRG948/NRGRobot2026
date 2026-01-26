/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.parameters.Colors.BLACK;
import static frc.robot.parameters.Colors.BLUE;
import static frc.robot.parameters.Colors.GREEN;
import static frc.robot.parameters.Colors.RED;
import static frc.robot.parameters.Colors.WHITE;
import static frc.robot.parameters.Colors.YELLOW;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.parameters.Colors;
import frc.robot.subsystems.StatusLED;
import frc.robot.subsystems.Subsystems;
import java.util.function.BooleanSupplier;

/** A namespace for LED command factory methods. */
public final class LEDCommands {

  /**
   * Returns a command that sets the color of the status LEDs.
   *
   * @param statusLED The status LED subsystem.
   * @param color The color to set.
   * @return A command that sets the color of the status LED.
   */
  public static Command setColor(StatusLED statusLED, Colors color) {
    return Commands.runOnce(() -> statusLED.fillAndCommitColor(color), statusLED)
        .withName(String.format("SetColor(%s)", color.name()));
  }

  /*
   * Returns a command that sets the autonomous LEDs to Flame Cycle.
   *
   */
  public static Command autoLEDs(Subsystems subsystems) {
    StatusLED leds = subsystems.statusLEDs;
    return new FlameCycle(leds);
  }

  /*
   * Returns a command that sets the last 5 seconds of transitioning between different shifts to blinking pink.
   */
  public static Command setTransitionModeLED(Subsystems subsystems) {

    StatusLED leds = subsystems.statusLEDs;
    return new BlinkColor(leds, Colors.PINK)
        .asProxy()
        .withName("Transition LED for last 5 seconds excluding the final second shift change.");
  }

  /*
   * Returns a command that sets the last second of transitioning between different shifts to solid red.
   */
  public static Command setLastSecondTransitionModeLED(Subsystems subsystems) {

    StatusLED leds = subsystems.statusLEDs;

    return setColor(leds, RED)
        .andThen(Commands.idle(leds))
        .withName("Transitional LED for last second shift change.");
  }

  /**
   * Returns a command that either flashes yellow for active mode and blinks red and blue for
   * inactive mode during shifts excluding shifting to endgame.
   *
   * @param isActive A BooleanSupplier for whether the robot is in active or inactive.
   * @return A command that sets the color of the status LED.
   */
  public static Command setModeLED(Subsystems subsystems, BooleanSupplier isActive) {
    StatusLED leds = subsystems.statusLEDs;

    return Commands.either(
            new AlternateColor(leds, YELLOW, BLACK).asProxy(),
            new AlternateColor(leds, RED, BLUE).asProxy(),
            isActive)
        .withName("Mode LEDs");
  }

  /*
   * Returns a command that sets the last 5 seconds of transitioning to endgame to blinking black and white.
   */
  public static Command transitionToEndgameModeLED(Subsystems subsystems) {
    StatusLED statusLEDs = subsystems.statusLEDs;

    return new AlternateColor(statusLEDs, BLACK, WHITE)
        .asProxy()
        .withName("LED for Transitioning to Endgame");
  }

  /*
   * Returns a command that sets endgame lights to solid blue.
   */
  public static Command endgameLED(Subsystems subsystems) {
    StatusLED statusLEDs = subsystems.statusLEDs;

    return setColor(statusLEDs, BLUE)
        .asProxy()
        .until(() -> DriverStation.getMatchTime() <= 0.0)
        .withName("Endgame LEDs");
  }

  /**
   * Returns a command that sets the climbing phase LEDs of the robot. When the robot is in the
   * process of climbing, the LEDs will blink rainbow cycle. When fully climbed, the LEDs will
   * display solid rainbow cycle.
   *
   * @param subsystems The subsystems container providing access to the status LEDs.
   * @param isClimbing A supplier that returns true while the robot is actively climbing.
   * @param isClimbed A supplier that returns true once the robot has finished climbing.
   */
  public static Command setClimbModeLED(
      Subsystems subsystems, BooleanSupplier isClimbing, BooleanSupplier isClimbed) {
    StatusLED statusLEDs = subsystems.statusLEDs;

    return Commands.either(
            new RainbowCycle(statusLEDs), new BlinkingRainbowCycle(statusLEDs), isClimbed)
        .onlyWhile(() -> isClimbed.getAsBoolean() || isClimbing.getAsBoolean())
        .withName("ClimbOverrideLEDs");
  }

  /*
   * Returns a command that sets LEDs to solid color green if the robot is aligned to the hub.
   */
  public static Command alignedLED(Subsystems subsystems, boolean isAligned) {
    StatusLED statusLED = subsystems.statusLEDs;

    return Commands.either(setColor(statusLED, GREEN), Commands.idle(statusLED), () -> isAligned);
  }

  /**
   * Returns a command that blinks the status LEDs red while a condition is true.
   *
   * @param subsystems The subsystems container.
   * @return A command that blinks the status LEDs red.
   */
  public static Command indicateErrorWithBlink(Subsystems subsystems) {
    return new BlinkColor(subsystems.statusLEDs, RED).withName("IndicateErrorWithBlink");
  }

  /**
   * Returns a command that turns the status LEDs red while a condition is true.
   *
   * @param subsystems The subsystems.
   * @return A command that turns the status LEDs red.
   */
  public static Command indicateErrorWithSolid(Subsystems subsystems) {
    StatusLED statusLEDs = subsystems.statusLEDs;

    return Commands.sequence(setColor(statusLEDs, RED), Commands.idle(statusLEDs))
        .withName("IndicateErrorWithSolid");
  }
}
