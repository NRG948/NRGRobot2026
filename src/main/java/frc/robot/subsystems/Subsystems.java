/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import com.nrg948.dashboard.annotations.DashboardTab;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotPreferences;
import frc.robot.util.MotorIdleMode;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

public class Subsystems {

  @DashboardTab(title = "Swerve")
  public final Swerve drivetrain = new Swerve();

  @DashboardTab(title = "Intake")
  public final Intake intake = new Intake();

  @DashboardTab(title = "IntakeArm")
  public final IntakeArm intakeArm = new IntakeArm();

  @DashboardTab(title = "Shooter")
  public final Shooter shooter = new Shooter();

  @DashboardTab(title = "Indexer")
  public final Indexer indexer = new Indexer();

  public final StatusLED statusLEDs = new StatusLED();

  // TODO: Add Cameras (need AprilTag subsystem)
  // TODO: Add Robot to camera transforms, names, and ports when preferences is implemented

  @DashboardTab(title = "Front Left Camera")
  public final Optional<AprilTag> frontLeftCamera =
      SubsystemsUtil.newOptionalSubsystem(
          AprilTag.class,
          RobotPreferences.APRIL_TAG.ENABLE_FRONT_LEFT,
          "FrontLeftCamera",
          new Transform3d(),
          8081);

  @DashboardTab(title = "Front Right Camera")
  public final Optional<AprilTag> frontRightCamera =
      SubsystemsUtil.newOptionalSubsystem(
          AprilTag.class,
          RobotPreferences.APRIL_TAG.ENABLE_FRONT_RIGHT,
          "FrontRightCamera",
          new Transform3d(),
          8080);

  private final Subsystem[] all;
  private final Subsystem[] manipulators;

  private Map<String, StringLogEntry> commandLogger;

  public Subsystems() {
    // Add all manipulator subsystems to the `manipulators` list.
    var manipulators = new ArrayList<Subsystem>(Arrays.asList(intake, shooter, indexer, intakeArm));

    // Add all non-manipulator subsystems to the `all` list.
    var all = new ArrayList<Subsystem>(Arrays.asList(drivetrain));

    frontLeftCamera.ifPresent(all::add);
    frontRightCamera.ifPresent(all::add);

    all.addAll(manipulators);
    this.all = all.toArray(Subsystem[]::new);
    this.manipulators = manipulators.toArray(Subsystem[]::new);

    commandLogger =
        Arrays.stream(this.all)
            .collect(
                Collectors.toMap(
                    Subsystem::getName,
                    s ->
                        new StringLogEntry(
                            DataLogManager.getLog(),
                            String.format("/%s/ActiveCommand", s.getName()))));

    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.onCommandInitialize(
        (cmd) -> {
          cmd.getRequirements().stream()
              .forEach((s) -> commandLogger.get(s.getName()).append(cmd.getName()));
        });
    scheduler.onCommandFinish(
        (cmd) -> {
          cmd.getRequirements().stream().forEach((s) -> commandLogger.get(s.getName()).append(""));
        });

    SubsystemsDashboardTabs.bind(this);
  }

  /** Returns an array of all subsystems. */
  public Subsystem[] getAll() {
    return all;
  }

  /** Returns an array of all manipulator subsystems. */
  public Subsystem[] getManipulators() {
    return manipulators;
  }

  public void setInitialStates() {}

  /** Disables the specified subsystems implementing the {@link ActiveSubsystem} interface. */
  private void disableSubsystems(Subsystem[] subsystems) {
    for (Subsystem subsystem : subsystems) {
      if (subsystem instanceof ActiveSubsystem activeSubsystem) {
        activeSubsystem.disable();
      }
    }
  }

  /** Disables all subsystems implementing the {@link ActiveSubsystem} interface. */
  public void disableAll() {
    disableSubsystems(all);
  }

  /** Disables all manipulator subsystems implementing the {@link ActiveSubsystem} interface. */
  public void disableManipulators() {
    disableSubsystems(manipulators);
  }

  /**
   * Sets the idle mode of the motors for all subsystems implementing the {@link ActiveSubsystem}
   * interface.
   */
  public void setIdleMode(MotorIdleMode idleMode) {
    for (Subsystem subsystem : all) {
      if (subsystem instanceof ActiveSubsystem activeSubsystem) {
        activeSubsystem.setIdleMode(idleMode);
      }
    }
  }

  /** Called to perform periodic actions. */
  public void periodic() {}

  // TODO add updateEstimatedPose function (need AprilTag subystem first)
}
