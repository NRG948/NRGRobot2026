/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */

package frc.robot.commands;

import java.io.File;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import com.nrg948.autonomous.Autonomous;
import com.nrg948.autonomous.AutonomousCommandGenerator;
import com.nrg948.autonomous.AutonomousCommandMethod;
import com.nrg948.dashboard.annotations.DashboardComboBoxChooser;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Subsystems;
import io.arxila.javatuples.LabelValue;

/**
 * Autos class for Elastic Tab. This includes choosing autos and auto
 * visualization.
 */
@DashboardDefinition
public final class Autos {
  private static final String AUTO_FILE_TYPE = ".auto";

  private static final File AUTOS_DIR = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");

  private static final HashMap<String, Command> autosMap = new HashMap<String, Command>();

  /** Initiates Dropdown Menu for autonomous routine. */
  @DashboardComboBoxChooser(title = "Routine", column = 0, row = 0, width = 2, height = 1)
  private final SendableChooser<Command> autoChooser;

  /** Returns an autonomous command that does nothing. */
  @AutonomousCommandMethod(name = "None", isDefault = true)
  public static Command none(Subsystems subsystem) {
    return Commands.none().withName("None");
  }

  /**
   * Initializes autoChooser for tab dependency via RobotContainer.
   *
   * @param container
   */
  public Autos(RobotContainer container) {
    autoChooser = Autonomous.getChooser(container, "frc.robot");
  }

  /**
   * Returns a collection of label-value pairs mapping autonomous routine names to
   * autonomous
   * commands define using Pathplannner.
   */
  @AutonomousCommandGenerator
  public static Collection<LabelValue<String, Command>> generatePathPlannerAutos(
      Subsystems subsystems) {
    return Arrays.stream(AUTOS_DIR.listFiles((file, name) -> name.endsWith(AUTO_FILE_TYPE)))
        .map((file) -> file.getName().split("\\.")[0])
        .sorted()
        .map(name -> new LabelValue<>(name, generatePathPlannerAuto(subsystems, name)))
        .toList();
  }

  /**
   * Returns the PathPlanner auto command.
   *
   * @param subsystems Subsystems container.
   * @param name       Name of the PathPlanner auto.
   * @return The PathPlanner auto command.
   */
  public static Command generatePathPlannerAuto(Subsystems subsystems, String name) {
    NamedCommands.registerCommands(getPathplannerEventMap(subsystems, name));

    Set<Subsystem> requirements = new HashSet<>(Arrays.asList(subsystems.getManipulators()));
    requirements.add(subsystems.drivetrain);
    return Commands.defer(() -> getPathPlannerAuto(name), requirements).withName(name);
  }

  

  /** Gets selected autonomous routine. */
  public Command getAutonomous() {
    return autoChooser.getSelected();
  }
}
