/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot;

import static frc.robot.commands.DriveCommands.hubAimAndXLock;
import static frc.robot.commands.IntakeCommands.extendAndIntakeWhenSafe;
import static frc.robot.commands.IntakeCommands.moveArmToAngle;
import static frc.robot.commands.ShootingCommands.rampUpShooter;
import static frc.robot.commands.ShootingCommands.rampUpShooterForHub;
import static frc.robot.commands.ShootingCommands.shootWhenInRange;
import static frc.robot.commands.ShootingCommands.shootWhenInRangeAndOnShift;
import static frc.robot.subsystems.IntakeArm.BUMP_ANGLE;
import static frc.robot.subsystems.IntakeArm.EXTENDED_ANGLE;
import static frc.robot.subsystems.IntakeArm.HALF_STOW_ANGLE;
import static frc.robot.subsystems.IntakeArm.STOW_ANGLE;

import com.nrg948.dashboard.annotations.DashboardTab;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveAutoRotation;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveUsingController;
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.LEDs.FlameCycle;
import frc.robot.commands.LEDs.LEDCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.MatchUtil;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController manipulatorController =
      new CommandXboxController(OperatorConstants.MANIPULATOR_CONTROLLER_PORT);

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  private static final DataLog LOG = DataLogManager.getLog();

  private final BooleanLogEntry logDriverConnected =
      new BooleanLogEntry(LOG, "/Operator/Controller/Driver/Connected");
  private final BooleanLogEntry logManipConnected =
      new BooleanLogEntry(LOG, "/Operator/Controller/Manipulator/Connected");

  @DashboardTab(
      title = "Operator",
      modes = {"Competition", "Testing"})
  private final RobotOperator operator;

  @DashboardTab(
      title = "Preferences",
      modes = {"Competition", "Testing"})
  private final RobotPreferences preferences = new RobotPreferences();

  private final Subsystems subsystems = new Subsystems();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    Autos.init(subsystems);

    operator = new RobotOperator(subsystems, driverController, manipulatorController);

    subsystems.drivetrain.setDefaultCommand(
        new DriveUsingController(subsystems.drivetrain, driverController));
    subsystems.intake.setDefaultCommand(Commands.idle(subsystems.intake).withName("Idle"));
    subsystems.intakeArm.setDefaultCommand(Commands.idle(subsystems.intakeArm).withName("Idle"));

    subsystems.statusLEDs.ifPresent(
        statusLEDS -> statusLEDS.setDefaultCommand(new FlameCycle(statusLEDS)));

    // Configure the trigger bindings
    configureBindings();

    RobotContainerDashboardTabs.bind(this);
    FollowPathCommand.warmupCommand().schedule();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Swerve drivetrain = subsystems.drivetrain;

    new Trigger(MatchUtil::isAutonomous).whileTrue(LEDCommands.autoLEDs(subsystems));

    driverController
        .a()
        .whileTrue(
            Commands.parallel(
                    new DriveAutoRotation(drivetrain, driverController),
                    shootWhenInRange(subsystems))
                .withName("AutoAlignAndShootWhenInRange"));

    driverController
        .b()
        .whileTrue(
            Commands.parallel(
                    new DriveAutoRotation(drivetrain, driverController),
                    shootWhenInRangeAndOnShift(subsystems))
                .withName("AutoAlignAndShootWhenInRangeAndOnShift"));

    driverController
        .x()
        .whileTrue(
            Commands.parallel(
                    hubAimAndXLock(drivetrain, driverController).repeatedly(),
                    shootWhenInRange(subsystems))
                .withName("AutoAlignAndShootWithXLock"));

    driverController
        .povUp()
        .whileTrue(
            Commands.parallel(
                    Commands.run(drivetrain::setXLock, drivetrain),
                    ShootingCommands.shootFromHub(subsystems))
                .withName("ManualShootFromHub"));
    driverController
        .povDown()
        .whileTrue(
            Commands.parallel(
                    Commands.run(drivetrain::setXLock, drivetrain),
                    ShootingCommands.shootFromTower(subsystems))
                .withName("ManualShootFromTower"));

    driverController.leftTrigger().onTrue(moveArmToAngle(subsystems, BUMP_ANGLE));
    driverController.leftTrigger().onFalse(moveArmToAngle(subsystems, EXTENDED_ANGLE));
    driverController.rightBumper().whileTrue(extendAndIntakeWhenSafe(subsystems));
    driverController.leftBumper().whileTrue(IntakeCommands.outtake(subsystems));

    driverController.start().onTrue(DriveCommands.resetOrientation(subsystems));

    manipulatorController
        .povUp()
        .whileTrue(avoidConflicts(rampUpShooterForHub(subsystems), this::isDriverUsingShooter));
    manipulatorController.povDown().onTrue(moveArmToAngle(subsystems, HALF_STOW_ANGLE));

    manipulatorController.povLeft().whileTrue(IntakeCommands.intake(subsystems));
    manipulatorController
        .povRight()
        .whileTrue(avoidConflicts(rampUpShooter(subsystems), this::isDriverUsingShooter));

    manipulatorController
        .rightBumper()
        .whileTrue(
            avoidConflicts(extendAndIntakeWhenSafe(subsystems), this::isDriverUsingIntakeArm));

    manipulatorController.a().whileTrue(IntakeCommands.outtake(subsystems));
    manipulatorController.x().onTrue(moveArmToAngle(subsystems, STOW_ANGLE));
    manipulatorController.y().onTrue(moveArmToAngle(subsystems, BUMP_ANGLE));
    manipulatorController.b().onTrue(moveArmToAngle(subsystems, EXTENDED_ANGLE));

    manipulatorController.leftBumper().whileTrue(IndexerCommands.feed(subsystems));

    manipulatorController.back().onTrue(DriveCommands.interruptAll(subsystems));
  }

  /**
   * Conditionally returns a proxied version of the given command as long as the supplied conflict
   * condition is not met (e.g. another driver is not using the same mechanism.) If a conflict does
   * exist, returns {@link Commands.none()}.
   *
   * @param command the command to be proxied when no conflict is indicated.
   * @param hasConflict a boolean supplier that reports whether a conflict currently exists.
   * @return the command to run when there is no conflict, else a no-op command.
   */
  private Command avoidConflicts(Command command, BooleanSupplier hasConflict) {
    return Commands.either(Commands.none(), new ProxyCommand(command), hasConflict);
  }

  private boolean isDriverUsingShooter() {
    return driverController.a().getAsBoolean()
        || driverController.b().getAsBoolean()
        || driverController.x().getAsBoolean()
        || driverController.povUp().getAsBoolean()
        || driverController.povDown().getAsBoolean();
  }

  private boolean isDriverUsingIntakeArm() {
    return driverController.leftTrigger().getAsBoolean();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.getAutonomous();
  }

  public void disableInit() {
    subsystems.disableManipulators();
    // subsystems.setIdleMode(MotorIdleMode.COAST);
    // subsystems.drivetrain.setIdleMode(MotorIdleMode.COAST);
  }

  public void teleopInit() {
    // subsystems.drivetrain.setIdleMode(MotorIdleMode.BRAKE);
    // subsystems.intakeArm.setIdleMode(MotorIdleMode.BRAKE);

    operator.teleopInit();
  }

  public void autonomousInit() {
    subsystems.drivetrain.captureLevelBaseline();
    // subsystems.drivetrain.setIdleMode(MotorIdleMode.BRAKE);
    // subsystems.intakeArm.setIdleMode(MotorIdleMode.BRAKE);

    operator.autonomousInit();
  }

  public void periodic() {
    subsystems.periodic();
    operator.periodic();

    logDriverConnected.update(driverController.isConnected());
    logManipConnected.update(manipulatorController.isConnected());
  }
}
