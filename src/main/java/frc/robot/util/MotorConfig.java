/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import static frc.robot.util.MotorDirection.COUNTER_CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.COAST;

/**
 * A record that contains the motor configuration.
 *
 * @param direction The direction the motor rotates when a positive voltage is applied.
 * @param idleMode The motor behavior when idle (i.e. brake or coast mode).
 * @param distancePerRotation The distance the attached mechanism moves per rotation of the motor
 *     output shaft.
 *     <p>The unit of measure depends on the mechanism. For a mechanism that produces linear motion,
 *     the unit is typically in meters. For a mechanism that produces rotational motion, the unit is
 *     typically in radians.
 */
public record MotorConfig(
    MotorDirection direction, MotorIdleMode idleMode, double distancePerRotation) {
  /**
   * Constructs a MotorConfig with default values for any parameters that are null or zero. The
   * default values are as follows:
   *
   * <ul>
   *   <li>direction: COUNTER_CLOCKWISE_POSITIVE
   *   <li>idleMode: COAST
   *   <li>distancePerRotation: 1.0
   * </ul>
   *
   * @param direction The direction the motor rotates when a positive voltage is applied.
   * @param idleMode The motor behavior when idle (i.e. brake or coast mode).
   * @param distancePerRotation The distance the attached mechanism moves per rotation of the motor
   *     output shaft. The unit of measure depends on the mechanism. For a mechanism that produces
   *     linear motion, the unit is typically in meters. For a mechanism that produces rotational
   *     motion, the unit is typically in radians.
   */
  public MotorConfig {
    if (direction == null) {
      direction = COUNTER_CLOCKWISE_POSITIVE;
    }
    if (idleMode == null) {
      idleMode = COAST;
    }
    if (distancePerRotation == 0.0) {
      distancePerRotation = 1.0;
    }
  }
}
