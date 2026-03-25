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
import static frc.robot.commands.ShootingCommands.feedBallsToShooter;
import static frc.robot.commands.ShootingCommands.rampUpShooter;
import static frc.robot.commands.ShootingCommands.shootWhenInRange;
import static frc.robot.commands.ShootingCommands.shootWhenInRangeAndOnShift;
import static frc.robot.subsystems.IntakeArm.BUMP_ANGLE;
import static frc.robot.subsystems.IntakeArm.EXTENDED_ANGLE;
import static frc.robot.subsystems.IntakeArm.STOW_ANGLE;

import com.nrg948.dashboard.annotations.DashboardTab;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.BlinkColor;
import frc.robot.commands.DriveAutoRotation;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveUsingController;
import frc.robot.commands.FlameCycle;
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.LEDCommands;
import frc.robot.commands.ShootWhileMoving;
import frc.robot.commands.ShootingCommands;
import frc.robot.parameters.Colors;
import frc.robot.subsystems.StatusLED;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.HubState;
import frc.robot.util.MatchUtil;
import frc.robot.util.MotorIdleMode;
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

    operator = new RobotOperator(subsystems);

    subsystems.drivetrain.setDefaultCommand(
        new DriveUsingController(subsystems.drivetrain, driverController));

    subsystems.statusLEDs.setDefaultCommand(new FlameCycle(subsystems.statusLEDs));

    // Configure the trigger bindings
    configureBindings();

    RobotContainerDashboardTabs.bind(this);
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
    StatusLED statusLED = subsystems.statusLEDs;

    new Trigger(MatchUtil::isAutonomous).whileTrue(LEDCommands.autoLEDs(subsystems));

    new Trigger(() -> MatchUtil.isTeleop() && isHubState(HubState.ACTIVE))
        .whileTrue(LEDCommands.setColor(statusLED, Colors.GREEN));
    new Trigger(() -> isHubState(HubState.INACTIVE))
        .whileTrue(LEDCommands.setColor(statusLED, Colors.RED));
    new Trigger(() -> isHubState(HubState.PREPARING_SHOOTING_DISABLED))
        .whileTrue(new BlinkColor(statusLED, Colors.RED));
    new Trigger(() -> isHubState(HubState.PREPARING_SHOOTING_ENABLED))
        .whileTrue(new BlinkColor(statusLED, Colors.YELLOW));
    new Trigger(() -> isHubState(HubState.PREPARING_TO_DISABLE_10_SEC))
        .whileTrue(new BlinkColor(statusLED, Colors.YELLOW));
    new Trigger(() -> isHubState(HubState.NEARING_END_OF_MATCH))
        .whileTrue(new BlinkColor(statusLED, Colors.YELLOW));

    driverController
        .a()
        .whileTrue(
            Commands.parallel(
                new DriveAutoRotation(drivetrain, driverController), shootWhenInRange(subsystems)));

    driverController
        .b()
        .whileTrue(
            Commands.parallel(
                new DriveAutoRotation(drivetrain, driverController),
                shootWhenInRangeAndOnShift(subsystems)));

    driverController
        .x()
        .whileTrue(
            Commands.parallel(
                hubAimAndXLock(drivetrain, driverController).repeatedly(),
                shootWhenInRange(subsystems)));

    driverController
        .rightStick()
        .whileTrue(
            Commands.parallel(
                new ShootWhileMoving(subsystems, driverController),
                feedBallsToShooter(subsystems, true)));

    driverController.povUp().whileTrue(ShootingCommands.shootFromHub(subsystems));
    driverController.povDown().whileTrue(ShootingCommands.shootFromTower(subsystems));

    driverController.leftTrigger().onTrue(moveArmToAngle(subsystems, BUMP_ANGLE));
    driverController.leftTrigger().onFalse(moveArmToAngle(subsystems, EXTENDED_ANGLE));
    driverController.rightBumper().whileTrue(extendAndIntakeWhenSafe(subsystems));

    driverController.start().onTrue(DriveCommands.resetOrientation(subsystems));

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

  private boolean isHubState(HubState state) {
    return operator.getHubState() == state;
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
        || driverController.povDown().getAsBoolean()
        || driverController.rightStick().getAsBoolean();
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
    subsystems.setIdleMode(MotorIdleMode.COAST);
    subsystems.drivetrain.setIdleMode(MotorIdleMode.COAST);
    CommandScheduler.getInstance().schedule(LEDCommands.autoLEDs(subsystems));
  }

  public void teleopInit() {
    subsystems.drivetrain.setIdleMode(MotorIdleMode.BRAKE);
    subsystems.intakeArm.setIdleMode(MotorIdleMode.BRAKE);

    operator.teleopInit();
  }

  public void autonomousInit() {
    subsystems.drivetrain.setIdleMode(MotorIdleMode.BRAKE);
    subsystems.intakeArm.setIdleMode(MotorIdleMode.BRAKE);

    operator.autonomousInit();
  }

  public void periodic() {
    subsystems.periodic();
    operator.periodic();
  }
}
