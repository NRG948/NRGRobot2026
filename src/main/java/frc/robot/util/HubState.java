/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.nrg948.util.Colors;

/** An enum representing the hub states. */
public enum HubState {
  /** When the hub is not active, we are not ready to shoot. */
  INACTIVE(Colors.RED, false, 0),
  /** When the hub is nearing active, we are not ready to shoot. */
  PREPARING_SHOOTING_DISABLED(Colors.RED, true, 5),
  /** When the hub is nearing active, we are ready to shoot. */
  PREPARING_SHOOTING_ENABLED(Colors.YELLOW, true, 2),
  /** When the hub is active, we are ready to shoot. */
  ACTIVE(Colors.GREEN, false, 0),
  /** When the hub is nearing not active, we are ready to shoot. */
  PREPARING_TO_DISABLE_10_SEC(Colors.YELLOW, true, 10),
  /**
   * When the hub is nearing not active, we are ready to shoot. (But with a different color and
   * closer in time)
   */
  PREPARING_TO_DISABLE_5_SEC(Colors.ORANGE, true, 5),
  /** When nearing the end of the match. */
  NEARING_END_OF_MATCH(Colors.YELLOW, true, 5);

  private final String color;
  private final boolean blink;
  private final double deltaTime;

  private HubState(Colors color, boolean blink, double deltaTime) {
    this.color = color.toHexString();
    this.blink = blink;
    this.deltaTime = deltaTime;
  }

  public String getColor() {
    return color;
  }

  public boolean blink() {
    return blink;
  }

  public double getDeltaTime() {
    return deltaTime;
  }
}
