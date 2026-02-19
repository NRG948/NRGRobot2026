/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;

/** An interface for a sensor that provides rotational positioning. */
public interface Gyro extends Sendable, AutoCloseable {
  /** {@return the gyro yaw angle in radians} Positive values in the counter-clockwise direction. */
  double getYaw();

  /** {@return the gyro pitch angle in radians} Positive values for nose down. */
  double getPitch();

  /** {@return the gyro roll angle in radians} Positive values for robot right side down. */
  double getRoll();

  /** Resets the gyro to 0. */
  void reset();
}
