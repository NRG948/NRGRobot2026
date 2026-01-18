/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import com.nrg948.dashboard.annotations.DashboardTab;
import com.nrg948.preferences.RobotPreferences;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.MotorIdleMode;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class Subsystems {
  @DashboardTab public final Swerve drivetrain = new Swerve();

  // TODO: Add Cameras (need AprilTag subsystem)

  private final Subsystem[] all;
  private final Subsystem[] manipulators;

  private Map<String, StringLogEntry> commandLogger;

  public Subsystems() {
    // Add all manipulator subsystems to the `manipulators` list.
    var manipulators = new ArrayList<Subsystem>(Arrays.asList());

    // Add all non-manipulator subsystems to the `all` list.
    var all = new ArrayList<Subsystem>(Arrays.asList(drivetrain));

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
  }

  /** Returns an array of all subsystems. */
  public Subsystem[] getAll() {
    return all;
  }

  /** Returns an array of all manipulator subsystems. */
  public Subsystem[] getManipulators() {
    return manipulators;
  }

  /**
   * Creates a new optional subsystem.
   *
   * @param <T> The type of subsystem.
   * @param subsystemClass The subsystem class.
   * @param enabled The preferences value indicating whether the subsystem is enabled.
   * @param initArgs The arguments to pass to the subsystem's constructor.
   * @return Returns a non-empty {@link Optional} instance if the subsystem is enabled. Otherwise,
   *     this method returns {@link Optional#empty}.
   */
  private static <T extends Subsystem> Optional<T> newOptionalSubsystem(
      Class<T> subsystemClass, RobotPreferences.BooleanValue enabled, Object... initArgs) {
    if (!enabled.getValue()) {
      return Optional.empty();
    }

    Class<?>[] initArgClasses = Stream.of(initArgs).map(Object::getClass).toArray(Class<?>[]::new);

    try {
      return Optional.of(subsystemClass.getConstructor(initArgClasses).newInstance(initArgs));
    } catch (InstantiationException
        | IllegalAccessException
        | IllegalArgumentException
        | InvocationTargetException
        | SecurityException e) {
      System.err.printf(
          "ERROR: An unexpected exception was caught while creating an instance of %s.%n",
          subsystemClass.getName());
      e.printStackTrace();
      return Optional.empty();
    } catch (NoSuchMethodException e) {
      System.err.printf(
          "ERROR: The class is missing constructor %s(%s).%n",
          subsystemClass.getName(),
          Stream.of(initArgClasses).map(Class::getSimpleName).collect(Collectors.joining(", ")));
      e.printStackTrace();
      return Optional.empty();
    }
  }

  public void setInitialStates() {}

  /** Disables the specified subsystems implementing the {@link ActiveSubsystem} interface. */
  private void disableSubsystems(Subsystem[] subsystems) {
    for (Subsystem subsystem : subsystems) {
      if (subsystem instanceof ActiveSubsystem) {
        ActiveSubsystem.class.cast(subsystem).disable();
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
