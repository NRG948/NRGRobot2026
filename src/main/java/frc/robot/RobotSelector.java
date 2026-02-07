/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot;

import com.nrg948.util.enums.SelectorEnum;

/** Defines the types of robots that can be selected. */
public enum RobotSelector implements SelectorEnum<RobotSelector> {
  /** The practice robot for 2026. */
  PracticeRobot2026,
  /** The competition robot for 2026. */
  CompetitionRobot2026,
  /** The alpha robot for 2026. */
  AlphaRobot2026;
}
