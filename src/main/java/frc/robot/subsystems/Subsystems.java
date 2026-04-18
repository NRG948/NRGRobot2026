/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.RobotPreferences.isCompBot;

import com.nrg948.dashboard.annotations.DashboardTab;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.RobotConstants.CANID;
import frc.robot.RobotPreferences;
import frc.robot.util.MotorCurrentConfig;
import frc.robot.util.MotorIdleMode;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

public final class Subsystems {

  @DashboardTab(
      title = "Swerve",
      modes = {"Testing"})
  public final Swerve drivetrain = new Swerve();

  @DashboardTab(
      title = "Intake",
      modes = {"Testing"})
  public final Intake intake = new Intake();

  @DashboardTab(
      title = "IntakeArm",
      modes = {"Testing"})
  public final IntakeArm intakeArm = new IntakeArm();

  @DashboardTab(
      title = "Shooter",
      modes = {"Testing"})
  public final Shooter shooter = new Shooter();

  private static final double ROLLER_DIAMETER = Units.inchesToMeters(1.25);

  private static final MotorCurrentConfig INDEXER_CURRENT_CONFIG =
      new MotorCurrentConfig(60.0, 60.0, true);
  private static final double INDEXER_GEAR_RATIO = isCompBot() ? 3.0 : 1.0;
  private static final double INDEXER_METERS_PER_REVOLUTION =
      (ROLLER_DIAMETER * Math.PI) / INDEXER_GEAR_RATIO;

  @DashboardTab(
      title = "Indexer",
      modes = {"Testing"})
  public final Rollers indexer =
      new Rollers(
          "Indexer",
          CANID.SHOOTER_INDEXER_ID,
          INDEXER_METERS_PER_REVOLUTION,
          INDEXER_CURRENT_CONFIG);

  private static final MotorCurrentConfig HOPPER_CURRENT_CONFIG =
      new MotorCurrentConfig(60.0, 60.0, true);
  private static final double HOPPER_GEAR_RATIO = isCompBot() ? 3.0 : 1.0;
  private static final double HOPPER_METERS_PER_REVOLUTION =
      (ROLLER_DIAMETER * Math.PI) / HOPPER_GEAR_RATIO;

  @DashboardTab(
      title = "Hopper",
      modes = {"Testing"})
  public final Rollers hopper =
      new Rollers(
          "Hopper", CANID.HOPPER_INDEXER_ID, HOPPER_METERS_PER_REVOLUTION, HOPPER_CURRENT_CONFIG);

  public final StatusLED statusLEDs = new StatusLED();

  @DashboardTab(
      title = "Front Left Camera",
      modes = {"Testing"})
  public final Optional<AprilTag> frontLeftCamera =
      AprilTag.PARAMETERS
          .frontLeft()
          .flatMap(
              (c) ->
                  SubsystemsUtil.newOptionalSubsystem(
                      AprilTag.class,
                      RobotPreferences.APRIL_TAG.ENABLE_FRONT_LEFT,
                      c.cameraName(),
                      c.robotToCamera(),
                      c.cameraPublisherName(),
                      c.streamURL()));

  @DashboardTab(
      title = "Front Right Camera",
      modes = {"Testing"})
  public final Optional<AprilTag> frontRightCamera =
      AprilTag.PARAMETERS
          .frontRight()
          .flatMap(
              (c) ->
                  SubsystemsUtil.newOptionalSubsystem(
                      AprilTag.class,
                      RobotPreferences.APRIL_TAG.ENABLE_FRONT_RIGHT,
                      c.cameraName(),
                      c.robotToCamera(),
                      c.cameraPublisherName(),
                      c.streamURL()));

  @DashboardTab(
      title = "Back Left Camera",
      modes = {"Testing"})
  public final Optional<AprilTag> backLeftCamera =
      AprilTag.PARAMETERS
          .backLeft()
          .flatMap(
              (c) ->
                  SubsystemsUtil.newOptionalSubsystem(
                      AprilTag.class,
                      RobotPreferences.APRIL_TAG.ENABLE_BACK_LEFT,
                      c.cameraName(),
                      c.robotToCamera(),
                      c.cameraPublisherName(),
                      c.streamURL()));

  @DashboardTab(
      title = "Back Right Camera",
      modes = {"Testing"})
  public final Optional<AprilTag> backRightCamera =
      AprilTag.PARAMETERS
          .backRight()
          .flatMap(
              (c) ->
                  SubsystemsUtil.newOptionalSubsystem(
                      AprilTag.class,
                      RobotPreferences.APRIL_TAG.ENABLE_BACK_RIGHT,
                      c.cameraName(),
                      c.robotToCamera(),
                      c.cameraPublisherName(),
                      c.streamURL()));

  private final Subsystem[] all;
  private final Subsystem[] manipulators;

  private Map<String, StringLogEntry> commandLogger;

  public Subsystems() {
    // Add all manipulator subsystems to the `manipulators` list.
    var manipulators =
        new ArrayList<Subsystem>(Arrays.asList(intake, shooter, indexer, hopper, intakeArm));

    var all = new ArrayList<Subsystem>(Arrays.asList(drivetrain, statusLEDs));

    frontLeftCamera.ifPresent(all::add);
    frontRightCamera.ifPresent(all::add);
    backLeftCamera.ifPresent(all::add);
    backRightCamera.ifPresent(all::add);

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

  /** {@return whether the front left camera is connected} */
  public boolean frontLeftCameraIsConnected() {
    return frontLeftCamera.map((c) -> c.isCameraConnected()).orElse(false);
  }

  /** {@return whether the front right camera is connected} */
  public boolean frontRightCameraIsConnected() {
    return frontRightCamera.map((c) -> c.isCameraConnected()).orElse(false);
  }

  /** {@return true if at least one camera is connected; false otherwise} */
  public boolean atLeastOneCameraConnected() {
    return frontLeftCameraIsConnected() || frontRightCameraIsConnected();
  }

  /** Called to perform periodic actions. */
  public void periodic() {
    frontRightCamera.ifPresent(this::updateEstimatedPose);
    frontLeftCamera.ifPresent(this::updateEstimatedPose);
    backLeftCamera.ifPresent(this::updateEstimatedPose);
    backRightCamera.ifPresent(this::updateEstimatedPose);
  }

  private void updateEstimatedPose(AprilTag camera) {
    var visionEst = camera.getEstimatedGlobalPose();

    visionEst.ifPresent(
        (est) -> {
          var estPose = est.estimatedPose.toPose2d();
          var estStdDevs = camera.getEstimationStdDevs();

          drivetrain.addVisionMeasurement(estPose, est.timestampSeconds, estStdDevs);
          if (camera.shouldUpdateOdometry()) {
            drivetrain.resetPosition(drivetrain.getPosition());
          }
        });
  }
}
