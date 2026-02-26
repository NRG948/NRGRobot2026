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
import com.nrg948.util.enums.EnumChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.parameters.AutoSide;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.MatchUtil;
import io.arxila.javatuples.LabelValue;
import io.arxila.javatuples.Pair;
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

  private static final HashMap<Pair<String, AutoSide>, Command> autosMap = new HashMap<>();

  private static final SendableChooser<AutoSide> sideChooser =
      EnumChooser.fromDefault(AutoSide.RIGHT);
  private static SendableChooser<Command> autoChooser;
  private static final SendableChooser<Integer> delayChooser = new SendableChooser<>();

  private static final Alert invalidAutoAlert =
      new Alert("Invalid auto combination. No auto will run.", AlertType.kError);

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
        MatchUtil::isRedAlliance,
        subsystems.drivetrain);

    autoChooser = Autonomous.getChooser(subsystems);
    autoChooser.onChange(Autos::preloadAuto);
    sideChooser.onChange(Autos::preloadAuto);

    delayChooser.setDefaultOption("No Delay", (Integer) 0);
    for (var i = 1; i < 8; i++) {
      delayChooser.addOption(String.format("%d Second Delay", i), (Integer) i);
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
    return Commands.defer(
            () ->
                Commands.sequence(
                    Commands.waitSeconds(delayChooser.getSelected()), getPathPlannerAuto(name)),
            requirements)
        .withName(name);
  }

  /**
   * Returns the PathPlanner auto command for the side selected by {@link #sideChooser} from the
   * {@link #autosMap}, creating one if it hasn't already been preloaded.
   *
   * @param name Name of the PathPlanner auto.
   * @return The PathPlanner auto command.
   */
  private static Command getPathPlannerAuto(String name) {
    AutoSide side = sideChooser.getSelected();
    Command autoCommand = autosMap.remove(new Pair<>(name, side));

    if (autoCommand == null) {
      autoCommand = newPathPlannerAuto(name, side);
    }

    return autoCommand;
  }

  /**
   * Preloads the specified PathPlanner command for the side selected by {@link #sideChooser}.
   *
   * @param auto The auto to preload.
   */
  private static void preloadAuto(Command auto) {
    if (auto == null) {
      return;
    }

    String autoName = auto.getName();
    preloadAuto(autoName, sideChooser.getSelected());
  }

  /**
   * Preloads the PathPlanner command selected by {@link #autoChooser} for the given side.
   *
   * @param side the side to preload the auto for
   */
  private static void preloadAuto(AutoSide side) {
    Command auto = autoChooser.getSelected();

    if (auto == null) {
      return;
    }

    preloadAuto(auto.getName(), side);
  }

  /**
   * Preloads the PathPlanner command with the given name for the given side.
   *
   * @param autoName the name of the PathPlanner auto
   * @param side the side to preload the auto for
   */
  private static void preloadAuto(String autoName, AutoSide side) {
    File autoFile = new File(AUTOS_DIR, autoName + AUTO_FILE_TYPE);

    if (autoFile.exists()) {
      Command autoCommand = newPathPlannerAuto(autoName, side);
      autosMap.put(new Pair<String, AutoSide>(autoName, side), autoCommand);
    }
  }

  /**
   * Checks if the specified PathPlanner auto is valid for the given side.
   *
   * @param autoName the name of the PathPlanner auto
   * @param side the side to check the auto for
   * @return true if the auto is valid, false otherwise
   */
  private static boolean isValidAuto(String autoName, AutoSide side) {
    return !autoName.contains("Outpost") || side == AutoSide.RIGHT;
  }

  /**
   * {@return a {@link PathPlannerAuto} instance for the given PathPlanner autonomous routine name}
   * If the selected PathPlanner routine is not valid for the selected side, returns a command that
   * does nothing.
   *
   * @param name the PathPlanner autonomous routine name
   * @param side the starting side/field position to run the auto from (e.g., {@code AutoSide.LEFT}
   *     or {@code AutoSide.RIGHT}); paths are mirrored when {@code side == AutoSide.LEFT}
   */
  private static Command newPathPlannerAuto(String name, AutoSide side) {
    if (!isValidAuto(name, side)) {
      invalidAutoAlert.set(true);

      return Commands.none().withName(name + " (Invalid)");
    }

    invalidAutoAlert.set(false);

    return new PathPlannerAuto(name, side == AutoSide.LEFT);
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
        Commands.parallel(
            IntakeCommands.setIntakeArmAngle(subsystems, IntakeArm.BUMP_ANGLE),
            IntakeCommands.disableIntake(subsystems)));

    eventMaps.put(
        "ExtendAndIntake",
        Commands.parallel(
            IntakeCommands.setIntakeArmAngle(subsystems, IntakeArm.EXTENDED_ANGLE),
            IntakeCommands.intake(subsystems)));

    eventMaps.put("Intake", IntakeCommands.intake(subsystems));

    eventMaps.put("Extend", IntakeCommands.setIntakeArmAngle(subsystems, IntakeArm.EXTENDED_ANGLE));

    eventMaps.put("DisableIntake", IntakeCommands.disableIntake(subsystems));

    return eventMaps;
  }

  /** {@return the selected autonomous routine} */
  public Command getAutonomous() {
    return autoChooser.getSelected();
  }

  /** {@return the {@link SendableChooser} for selecting the autonomous command} */
  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  /** {@return the {@link SendableChooser} for selecting the autonomous path starting side} */
  public SendableChooser<AutoSide> getSideChooser() {
    return sideChooser;
  }

  /** {@return the {@link SendableChooser} for selecting the delay at the start of autonomous} */
  public SendableChooser<Integer> getDelayChooser() {
    return delayChooser;
  }

  public Alert getInvalidAutoAlert() {
    return invalidAutoAlert;
  }
}
