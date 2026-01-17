/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.nrg948.autonomous.Autonomous;
import com.nrg948.autonomous.AutonomousCommandMethod;
import com.nrg948.dashboard.annotations.DashboardComboBoxChooser;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

/**
 * Autos class for Elastic Tab. This includes choosing autos and auto visualization.
 */
@DashboardDefinition
public final class Autos {

  /**
   * Initiates Dropdown Menu for autonomous routine. 
   */
  @DashboardComboBoxChooser(title = "Routine", column = 0, row = 0, width = 2, height = 1)
  private final SendableChooser<Command> autoChooser;

  /**
   * Default auto behavior to do nothing.
   * @param container
   */
  @AutonomousCommandMethod(name = "None", isDefault = true)
  public static Command noneAuto(RobotContainer container) {
    return Commands.none();
  }

  /**
   * Initializes autoChooser for tab dependency via RobotContainer. 
   * @param container
   */
  public Autos(RobotContainer container) {
    autoChooser = Autonomous.getChooser(container, "frc.robot");
  }

  /**
   * Gets selected autonomous routine.
   */
  public Command getAutonomous() {
    return autoChooser.getSelected();
  }
}
