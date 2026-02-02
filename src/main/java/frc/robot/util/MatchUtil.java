/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;

/** Utility class for match-related information and timing. */
public final class MatchUtil {
  private static final double PRE_FIRST_SHIFT_START_TIME = 135.0;
  private static final double FIRST_SHIFT_START_TIME = 130.0;
  private static final double PRE_SECOND_SHIFT_START_TIME = 110.0;
  private static final double SECOND_SHIFT_START_TIME = 105.0;
  private static final double PRE_THIRD_SHIFT_START_TIME = 85.0;
  private static final double THIRD_SHIFT_START_TIME = 80.0;
  private static final double PRE_FOURTH_SHIFT_START_TIME = 60.0;
  private static final double FOURTH_SHIFT_START_TIME = 55.0;

  /** {@return the alliance the robot is on} */
  public static Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  /** {@return true if the robot is on the red alliance, false otherwise} */
  public static boolean isRedAlliance() {
    return getAlliance() == Alliance.Red;
  }

  /** {@return the current match time in seconds} */
  public static double getMatchTime() {
    return DriverStation.getMatchTime();
  }

  /** {@return true if the match is a competition or practice match, false otherwise} */
  public static boolean isCompetition() {
    return DriverStation.getMatchType() != MatchType.None;
  }

  /** {@return true if the robot is in teleoperated mode} */
  public static boolean isTeleop() {
    return DriverStation.isTeleop();
  }

  /** {@return true if the robot is in autonomous mode} */
  public static boolean isAutonomous() {
    return DriverStation.isAutonomous();
  }

  /** {@return the remaining time in autonomous mode} */
  public static double getAutoTimeRemaining() {
    return isTeleop() ? 0 : DriverStation.getMatchTime();
  }

  /** {@return the remaining time in teleoperated mode} */
  public static double getTeleopTimeRemaining() {
    return isAutonomous() ? 0 : DriverStation.getMatchTime();
  }

  /**
   * {@return true if the robot is within 5 seconds of transitional periods} The transition to
   * endgame is not included.
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

  /**
   * Special method of {@link #isNearShiftChange()} specialized for last 5 seconds before shift
   * change but excluding the final second.
   */
  public static boolean isNearShiftChangeExcludingFinalSecond() {
    if (!isTeleop()) {
      return false;
    }

    double time = DriverStation.getMatchTime();

    return (time <= PRE_FIRST_SHIFT_START_TIME && time > FIRST_SHIFT_START_TIME + 1.0)
        || (time <= PRE_SECOND_SHIFT_START_TIME && time > SECOND_SHIFT_START_TIME + 1.0)
        || (time <= PRE_THIRD_SHIFT_START_TIME && time > THIRD_SHIFT_START_TIME + 1.0)
        || (time <= PRE_FOURTH_SHIFT_START_TIME && time > FOURTH_SHIFT_START_TIME + 1.0);
  }

  /**
   * Special method of {@link #isNearShiftChange()} specialized for the last second before shift
   * change.
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

  /** {@return whether there is 5 seconds or less before endgame} */
  public static boolean isNearEndgame() {
    if (!isTeleop()) {
      return false;
    }

    double time = DriverStation.getMatchTime();

    return (time <= 35.0 && time >= 30.0);
  }

  /** {@return whether the match is in endgame mode} */
  public static boolean isEndgame() {
    if (!isTeleop()) {
      return false;
    }

    return DriverStation.getMatchTime() <= 30.0;
  }

  /** {@return true if our alliance hub is active first} */
  public static boolean ourAllianceHubIsActiveFirst() {
    String gameData = DriverStation.getGameSpecificMessage();
    Alliance alliance = getAlliance();

    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          return alliance == Alliance.Blue;
        case 'R':
          return alliance == Alliance.Red;
        default:
          break;
      }
    }

    return false;
  }

  /**
   * {@return whether the hub is active based on the current match time and game data} The hub is
   * active during autonomous, and then shifts between being active and inactive during teleop based
   * on the match time and game data.
   */
  public static boolean isHubActive() {
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }

    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = ourAllianceHubIsActiveFirst();

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  private MatchUtil() {
    throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
  }
}
