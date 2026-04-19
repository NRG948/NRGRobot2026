/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

/**
 * A record that contains the motor current configuration.
 *
 * @param supplyCurrentLimit The supply current limit in amps. This is the maximum current that the
 *     motor controller will draw from the battery. Setting this limit can help prevent brownouts
 *     and protect the battery.
 * @param statorCurrentLimit The stator current limit in amps. This is the maximum current that the
 *     motor's stator windings can draw. Setting this limit can help prevent motor overheating and
 *     damage.
 * @param enableCurrentLimit A boolean indicating whether the current limits are enabled. If false,
 *     the motor controller will not enforce the current limits, and the motor may draw as much
 *     current as it needs to achieve the desired output. Enabling current limits can help protect
 *     the motor and battery, but may also reduce performance if the limits are set too low.
 */
public record MotorCurrentConfig(
    double supplyCurrentLimit, double statorCurrentLimit, boolean enableCurrentLimit) {
  public static final double DEFAULT_SUPPLY_CURRENT_LIMIT = 70.0;
  public static final double DEFAULT_STATOR_CURRENT_LIMIT = 120.0;

  /**
   * Creates a new MotorCurrentConfig with the specified parameters. If any of the current limits
   * are set to 0, they will be replaced with the default values.
   *
   * @param supplyCurrentLimit The supply current limit in amps.
   * @param statorCurrentLimit The stator current limit in amps.
   * @param enableCurrentLimit A boolean indicating whether the current limits are enabled.
   */
  public MotorCurrentConfig {
    if (supplyCurrentLimit == 0.0) {
      supplyCurrentLimit = DEFAULT_SUPPLY_CURRENT_LIMIT;
    }
    if (statorCurrentLimit == 0.0) {
      statorCurrentLimit = DEFAULT_STATOR_CURRENT_LIMIT;
    }
  }

  /**
   * Creates a new MotorCurrentConfig with the default current limits and current limiting enabled.
   */
  public MotorCurrentConfig() {
    this(DEFAULT_SUPPLY_CURRENT_LIMIT, DEFAULT_STATOR_CURRENT_LIMIT, true);
  }

  /**
   * Creates a new MotorCurrentConfig with the specified current limits and current limiting
   * enabled.
   *
   * @param supplyCurrentLimit The supply current limit in amps.
   * @param statorCurrentLimit The stator current limit in amps.
   */
  public MotorCurrentConfig(double supplyCurrentLimit, double statorCurrentLimit) {
    this(supplyCurrentLimit, statorCurrentLimit, true);
  }

  /**
   * Creates a new MotorCurrentConfig with the default current limits and the specified current
   * limiting state.
   *
   * @param enableCurrentLimit A boolean indicating whether the current limits are enabled.
   */
  public MotorCurrentConfig(boolean enableCurrentLimit) {
    this(DEFAULT_SUPPLY_CURRENT_LIMIT, DEFAULT_STATOR_CURRENT_LIMIT, enableCurrentLimit);
  }
}
