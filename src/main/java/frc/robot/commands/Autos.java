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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.FieldUtils;
import io.arxila.javatuples.LabelValue;
import java.io.File;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/** A utility class for autonomous related commands and selection. */
public final class Autos {
  private static final String AUTO_FILE_TYPE = ".auto";
  private static final File AUTOS_DIR =
      new File(Filesystem.getDeployDirectory(), "pathplanner/autos");

  private static final HashMap<String, Command> autosMap = new HashMap<String, Command>();

  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Integer> delayChooser = new SendableChooser<>();

  /**
   * Initializes autoChooser for tab dependency via RobotContainer.
   *
   * @param container
   */
  public Autos(Subsystems subsystems) {
    NamedCommands.registerCommands(getPathplannerEventMap(subsystems));
    RobotConfig config = Swerve.PARAMETERS.getPathplannerConfig();
    AutoBuilder.configure(
        subsystems.drivetrain::getPosition,
        subsystems.drivetrain::resetPosition,
        subsystems.drivetrain::getChassisSpeeds,
        subsystems.drivetrain::setChassisSpeeds,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        config,
        FieldUtils::isRedAlliance,
        subsystems.drivetrain);

    autoChooser = Autonomous.getChooser(subsystems);
    autoChooser.onChange(Autos::preloadAuto);

    this.delayChooser.setDefaultOption("No Delay", (Integer) 0);
    for (var i = 1; i < 8; i++) {
      this.delayChooser.addOption(String.format("%d Second Delay", i), (Integer) i);
    }
  }

  /** {@return an autonomous command that does nothing} */
  @AutonomousCommandMethod(name = "None", isDefault = true)
  public static Command none(Subsystems subsystems) {
    return Commands.none().withName("None");
  }

  /**
   * {@return a collection of label-value pairs mapping autonomous routine names to autonomous
   * commands defined using Pathplannner}
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
   * {@return the PathPlanner auto command}
   *
   * @param subsystems Subsystems container.
   * @param name Name of the PathPlanner auto.
   */
  public static Command generatePathPlannerAuto(Subsystems subsystems, String name) {
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
   * {@return a {@link PathPlannerAuto} instance for the given PathPlanner autonomous routine name}
   *
   * @param name the PathPlanner autonomous routine name
   */
  private static Command newPathPlannerAuto(String name) {
    return new PathPlannerAuto(name);
  }

  /**
   * Returns a map of event names to commands for the given Pathplanner autonomous routine name.
   *
   * @param subsystems Subsystems container.
   * @return A map of event names to commands.
   */
  private static Map<String, Command> getPathplannerEventMap(Subsystems subsystems) {

    Map<String, Command> eventMaps = new HashMap<String, Command>();
    eventMaps.put(
        "ShootWithAutoRotation",
        Commands.parallel(
            ShootingCommands.shoot(subsystems), new AutoRotation(subsystems.drivetrain)));

    eventMaps.put(
        "IntakeArmBumpAngle",
        Commands.sequence(
            IntakeCommands.setIntakeArmAngle(IntakeArm.BUMP_ANGLE, subsystems),
            IntakeCommands.disableIntake(subsystems)));

    eventMaps.put(
        "ExtendAndIntake",
        Commands.sequence(
            IntakeCommands.setIntakeArmAngle(IntakeArm.EXTENDED_ANGLE, subsystems),
            IntakeCommands.intake(subsystems)));

    eventMaps.put("Intake", IntakeCommands.intake(subsystems));

    eventMaps.put("Extend", IntakeCommands.setIntakeArmAngle(IntakeArm.EXTENDED_ANGLE, subsystems));

    eventMaps.put("DisableIntake", IntakeCommands.disableIntake(subsystems));

    return eventMaps;
  }

  /** {@return the selected autonomous routine} */
  public Command getAutonomous() {
    return Commands.sequence(
        Commands.waitSeconds(this.delayChooser.getSelected()), this.autoChooser.getSelected());
  }

  /** {@return the {@link SendableChooser} for selecting the autonomous command} */
  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  /** {@return the {@link SendableChooser} for selecting the delay at the start of autonomous} */
  public SendableChooser<Integer> getDelayChooser() {
    return delayChooser;
  }
}
