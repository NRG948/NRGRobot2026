/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

public enum ElevatorLevel {
  // Levels: Stowed, Level 1, Level 2, Level 3
  // TODO: add heights for every level
  STOWED(0),
  L1(0),
  L2(0),
  L3(0);

  // variables
  private final double elevatorHeight;

  // constructor
  private ElevatorLevel(double height) {
    this.elevatorHeight = height;
  }

  // getters
  public double getElevatorHeight() {
    return elevatorHeight;
  }
}
