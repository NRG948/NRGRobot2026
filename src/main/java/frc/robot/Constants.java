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
    public static final int kDriverControllerPort = 0;
  }

  public static class RobotConstants {
    public static final double MAX_BATTERY_VOLTAGE = 12.0;
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.9);

    public static final class CANID {
      public static final int INTAKE_ID = 9;
      public static final int INTAKE_ARM_ID = 10;
      public static final int SHOOTER_LOWER_1_ID = 11;
      public static final int SHOOTER_LOWER_2_ID = 12;
      public static final int SHOOTER_UPPER_1_ID = 13;
      public static final int SHOOTER_UPPER_2_ID = 14;
    }
  }
}
