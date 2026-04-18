/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

/** An exception thrown whenever we fail to configure a motor. */
public class MotorConfigException extends Exception {
  public MotorConfigException(String message) {
    super(message);
  }
}
