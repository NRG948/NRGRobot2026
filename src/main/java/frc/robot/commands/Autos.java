/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.nrg948.autonomous.Autonomous;
import com.nrg948.autonomous.AutonomousCommandGenerator;
import com.nrg948.autonomous.AutonomousCommandMethod;
import com.nrg948.dashboard.annotations.DashboardComboBoxChooser;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Subsystems;
import io.arxila.javatuples.LabelValue;
import java.io.File;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/** Autos class for Elastic Tab. This includes choosing autos and auto visualization. */
@DashboardDefinition
public final class Autos {
  private static final String AUTO_FILE_TYPE = ".auto";

  private static final File AUTOS_DIR =
      new File(Filesystem.getDeployDirectory(), "pathplanner/autos");

  private static final HashMap<String, Command> autosMap = new HashMap<String, Command>();

  /** Initiates Dropdown Menu for autonomous routine. */
  @DashboardComboBoxChooser(title = "Routine", column = 0, row = 0, width = 2, height = 1)
  private final SendableChooser<Command> autoChooser;

  /** Returns an autonomous command that does nothing. */
  @AutonomousCommandMethod(name = "None", isDefault = true)
  public static Command none(Subsystems subsystems) {
    return Commands.none().withName("None");
  }

  /**
   * Initializes autoChooser for tab dependency via RobotContainer.
   *
   * @param container
   */
  public Autos(Subsystems subsystems) {
    autoChooser = Autonomous.getChooser(subsystems, "frc.robot");
  }

  /**
   * Returns a collection of label-value pairs mapping autonomous routine names to autonomous
   * commands defined using Pathplannner.
   */
  @AutonomousCommandGenerator
  public static Collection<LabelValue<String, Command>> generatePathPlannerAutos(
      Subsystems subsystems) {
    File[] autoFiles = AUTOS_DIR.listFiles((file, name) -> name.endsWith(AUTO_FILE_TYPE));
    if (autoFiles == null) {
      return java.util.List.of();
    }
    return Arrays.stream(autoFiles)
        .map((file) -> file.getName().split("\\.")[0])
        .sorted()
        .map(name -> new LabelValue<>(name, generatePathPlannerAuto(subsystems, name)))
        .toList();
  }

  /**
   * Returns the PathPlanner auto command.
   *
   * @param subsystems Subsystems container.
   * @param name Name of the PathPlanner auto.
   * @return The PathPlanner auto command.
   */
  public static Command generatePathPlannerAuto(Subsystems subsystems, String name) {
    NamedCommands.registerCommands(getPathplannerEventMap(subsystems, name));

    Set<Subsystem> requirements = new HashSet<>(Arrays.asList(subsystems.getManipulators()));
    requirements.add(subsystems.drivetrain);
    return Commands.defer(() -> getPathPlannerAuto(name), requirements).withName(name);
  }

  /**
   * Returns the PathPlanner auto command from the autosMap, creating one if it hasn't already been
   * preloaded.
   *
   * @param name Name of the PathPlanner auto.
   * @return The PathPlanner auto command.
   */
  private static Command getPathPlannerAuto(String name) {
    Command autoCommand = autosMap.remove(name);
    if (autoCommand == null) {
      autoCommand = newPathPlannerAuto(name);
    }
    return autoCommand;
  }

  /**
   * Preloads the specified PathPlanner command.
   *
   * @param auto The auto to preload.
   */
  public static void preloadAuto(Command auto) {
    if (auto == null) {
      return;
    }

    String autoName = auto.getName();
    File autoFile = new File(AUTOS_DIR, autoName + AUTO_FILE_TYPE);

    if (autoFile.exists()) {
      Command autoCommand = newPathPlannerAuto(autoName);
      autosMap.put(autoName, autoCommand);
    }
  }

  /**
   * Returns a {@link PathPlannerAuto} instance for the given Pathplanner autonomous routine name.
   */
  private static Command newPathPlannerAuto(String name) {
    return new PathPlannerAuto(name);
  }

  /**
   * Returns a map of event names to commands for the given Pathplanner autonomous routine name.
   *
   * @param subsystems Subsystems container.
   * @param pathGroupName Name of the pathplanner autonomous routine.
   * @return A map of event names to commands.
   */
  private static Map<String, Command> getPathplannerEventMap(
      Subsystems subsystems, String pathGroupName) {

    Map<String, Command> eventMaps = new HashMap<String, Command>();
    // TODO: Populate eventMaps with commands for PathPlanner event markers for the given
    // pathGroupName.
    return eventMaps;
  }

  /** Gets selected autonomous routine. */
  public Command getAutonomous() {
    return autoChooser.getSelected();
  }
}
