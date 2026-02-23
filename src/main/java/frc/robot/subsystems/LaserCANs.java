/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LaserCANConstants;
import frc.robot.util.LaserCANSensor;

/** Subsystem made of 2 LaserCANs. */
public final class LaserCANs extends SubsystemBase {

  private LaserCANSensor leftLaserCAN;
  private LaserCANSensor rightLaserCAN;

  private double leftDistance = LaserCANSensor.NO_MEASURMENT;
  private double rightDistance = LaserCANSensor.NO_MEASURMENT;

  /** Creates the LaserCANs subsystem made of 2 LaserCANs. */
  public LaserCANs() {
    leftLaserCAN =
        new LaserCANSensor(
            LaserCANConstants.LEFT_LASER_CAN_ID,
            "Left LaserCAN",
            LaserCANConstants.LEFT_DISTANCE_CORRECTION);
    rightLaserCAN =
        new LaserCANSensor(
            LaserCANConstants.RIGHT_LASER_CAN_ID,
            "Right LaserCAN",
            LaserCANConstants.RIGHT_DISTANCE_CORRECTION);
  }

  @Override
  public void periodic() {
    leftDistance = leftLaserCAN.getDistance();
    rightDistance = rightLaserCAN.getDistance();
  }

  public double getLeftDistance() {
    return leftDistance;
  }

  public double getRightDistance() {
    return rightDistance;
  }
}
