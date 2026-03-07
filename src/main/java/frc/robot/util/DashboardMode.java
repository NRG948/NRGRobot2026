/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

/** Enum representing the different dashboard modes. */
public enum DashboardMode {
  /** Competition mode. */
  COMPETITION("Competition"),
  /** Testing mode. */
  TESTING("Testing");

  private final String modeName;

  /** Constructor for DashboardMode enum. */
  DashboardMode(String modeName) {
    this.modeName = modeName;
  }

  @Override
  public String toString() {
    return modeName;
  }
}
