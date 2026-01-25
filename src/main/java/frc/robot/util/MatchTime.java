/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public class MatchTime {

  private static final double PRE_FIRST_SHIFT_START_TIME = 135.0;
  private static final double FIRST_SHIFT_START_TIME = 130.0;
  private static final double PRE_SECOND_SHIFT_START_TIME = 110.0;
  private static final double SECOND_SHIFT_START_TIME = 105.0;
  private static final double PRE_THIRD_SHIFT_START_TIME = 85.0;
  private static final double THIRD_SHIFT_START_TIME = 80.0;
  private static final double PRE_FOURTH_SHIFT_START_TIME = 60.0;
  private static final double FOURTH_SHIFT_START_TIME = 55.0;

  public static boolean isTeleop() {
    return DriverStation.isTeleop();
  }

  public static boolean isAutonomous() {
    return DriverStation.isAutonomous();
  }

  public static double getAutoTimeRemaining() {
    return isTeleop() ? 0 : DriverStation.getMatchTime();
  }

  public static double getTeleopTimeRemaining() {
    return isAutonomous() ? 0 : DriverStation.getMatchTime();
  }

  /**
   * Returns if robot is within 5 seconds of transitional periods (transition to endgame not
   * included)
   */
  public static boolean isNearShiftChange() {
    if (!isTeleop()) {
      return false;
    }
    double time = DriverStation.getMatchTime();
    return (time <= PRE_FIRST_SHIFT_START_TIME && time >= FIRST_SHIFT_START_TIME)
        || (time <= PRE_SECOND_SHIFT_START_TIME && time >= SECOND_SHIFT_START_TIME)
        || (time <= PRE_THIRD_SHIFT_START_TIME && time >= THIRD_SHIFT_START_TIME)
        || (time <= PRE_FOURTH_SHIFT_START_TIME && time >= FOURTH_SHIFT_START_TIME);
  }

  /*
   * Special method of isNearShiftChange specialized for last 5 seconds before shift change but excluding the final second.
   */
  public static boolean isNearShiftChangeExcludingFinalSecond() {
    if (!isTeleop()) {
      return false;
    }
    double time = DriverStation.getMatchTime();
    return (time <= PRE_FIRST_SHIFT_START_TIME && time >= FIRST_SHIFT_START_TIME - 1.0)
        || (time <= PRE_SECOND_SHIFT_START_TIME && time >= SECOND_SHIFT_START_TIME - 1.0)
        || (time <= PRE_THIRD_SHIFT_START_TIME && time >= THIRD_SHIFT_START_TIME - 1.0)
        || (time <= PRE_FOURTH_SHIFT_START_TIME && time >= FOURTH_SHIFT_START_TIME - 1.0);
  }

  /*
   * Special method of isNearShiftChange speciaized for the last second before shift change.
   */
  public static boolean isNearShiftChangeFinalSecond() {
    if (!isTeleop()) {
      return false;
    }
    double time = DriverStation.getMatchTime();
    return (time <= FIRST_SHIFT_START_TIME + 1.0 && time >= FIRST_SHIFT_START_TIME)
        || (time <= SECOND_SHIFT_START_TIME + 1.0 && time >= SECOND_SHIFT_START_TIME)
        || (time <= THIRD_SHIFT_START_TIME + 1.0 && time >= THIRD_SHIFT_START_TIME)
        || (time <= FOURTH_SHIFT_START_TIME + 1.0 && time >= FOURTH_SHIFT_START_TIME);
  }

  /*
   * Returns a boolean to indicate if match time is 5 seconds or less before endgame.
   */
  public static boolean isNearEndgame() {
    if (!isTeleop()) {
      return false;
    }
    double time = DriverStation.getMatchTime();
    return (time <= 35.0 && time >= 30.0);
  }

  /*
   * Returns a boolean to indicate if match is in endgame mode.
   */
  public static boolean isEndgame() {
    if (!isTeleop()) {
      return false;
    }
    double time = DriverStation.getMatchTime();
    return time <= 30.0;
  }

  /**
   * Gets which team will go INACTIVE first. @Returns True if blue alliance go INACTIVE
   * first. @Returns False if red alliance go INACTIVE first.
   */
  public static boolean getFirstInactive() {
    String gameData;
    gameData = DriverStation.getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          return true;
        case 'R':
          return false;
        default:
          break;
      }
    }
    return false;
  }
}
