/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import frc.robot.subsystems.Climber;

public enum ElevatorLevel {
  // TODO: find actual values for STOWED, L1Latch, L1Climb
  STOWED(Climber.STOWED_HEIGHT_FOR_PID),
  L1_LATCH(0.15),
  L1_CLIMB(0.15);

  private final double elevatorHeight;

  /**
   * Constructs a variant of this enum.
   *
   * @param height The desired height in meters.
   */
  private ElevatorLevel(double height) {
    this.elevatorHeight = height;
  }

  /** Returns the desired height in meters. */
  public double getElevatorHeight() {
    return elevatorHeight;
  }
}
