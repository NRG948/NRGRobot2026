/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int MANIPULATOR_CONTROLLER_PORT = 1;
  }

  public static class RobotConstants {
    public static final double MAX_BATTERY_VOLTAGE = 12.0;
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.9);
    public static final int LED_COUNT = 77; // TODO: determine LED count

    public static class LEDSegment {
      public static final int STATUS_FIRST_LED = 0;
      public static final int STATUS_LED_COUNT = 56; // TODO: determine status LED count.
    }

    public static class PWMPort {
      public static final int LED = 1;
    }

    public static final class CANID {
      public static final int INTAKE_ID = 9;
      public static final int INTAKE_ARM_ID = 10;
      public static final int SHOOTER_LOWER_LEFT_ID = 13;
      public static final int SHOOTER_LOWER_RIGHT_ID = 14;
      public static final int SHOOTER_UPPER_LEFT_ID = 11;
      public static final int SHOOTER_UPPER_RIGHT_ID = 12;
      public static final int INDEXER_ID = 15;
    }

    public static final class CAN {
      public static final int LASER_CAN_ID = 0; // TODO: SET LASER CAN ID
    }
  }

  public static class VisionConstants {
    // TODO: Change this for the hub/tower
    /** The translational tolerance value for aligning. */
    public static final double POSE_ALIGNMENT_TOLERANCE_XY = 0.02; // in m

    /** The rotational tolerance value for aligning. */
    public static final double POSE_ALIGNMENT_TOLERANCE_R = 1.0; // in deg
  }
}
