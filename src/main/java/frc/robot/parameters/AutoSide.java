/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

public enum AutoSide {
  LEFT("Left"),
  RIGHT("Right");

  private String displayName;

  private AutoSide(String displayName) {
    this.displayName = displayName;
  }

  public String toString() {
    return displayName;
  }
}
