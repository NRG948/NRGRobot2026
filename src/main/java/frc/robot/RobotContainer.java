/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot;

import com.nrg948.dashboard.annotations.DashboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveUsingController;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.LEDCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.MatchTime;

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

  @DashboardTab(title = "Operator")
  private final RobotOperator operator;

  @DashboardTab(title = "Preferences")
  private final RobotPreferences preferences = new RobotPreferences();

  private final Subsystems subsystems = new Subsystems();

  @DashboardTab(title = "Autonomous")
  private final Autos autos = new Autos(subsystems);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    operator = new RobotOperator(subsystems);

    subsystems.drivetrain.setDefaultCommand(
        new DriveUsingController(subsystems.drivetrain, driverController));
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverController.start().onTrue(DriveCommands.resetOrientation(subsystems));

    new Trigger(MatchTime::isAutonomous).whileTrue(LEDCommands.autoLEDs(subsystems));
    new Trigger(MatchTime::isNearShiftChangeExcludingFinalSecond)
        .whileTrue(LEDCommands.setTransitionModeLED(subsystems));
    new Trigger(MatchTime::isNearShiftChangeFinalSecond)
        .whileTrue(LEDCommands.setLastSecondTransitionModeLED(subsystems));
    new Trigger(MatchTime::isNearEndgame)
        .whileTrue(LEDCommands.transitionToEndgameModeLED(subsystems));
    new Trigger(MatchTime::isEndgame).whileTrue(LEDCommands.endgameLED(subsystems));

    manipulatorController.rightBumper().whileTrue(IntakeCommands.intake(subsystems));
    manipulatorController.a().whileTrue(IntakeCommands.outtake(subsystems));

    // Experimental, remove after shooter interpolation table is made and implemented. Up and left
    // is increase and decrease upper shooter velocities respectively. Down and right is increase
    // and decrease lower shooter velocities respectively.
    manipulatorController
        .povUp()
        .onTrue(ShootingCommands.increaseShooterVelocityByPointTwo(subsystems));
    manipulatorController
        .povDown()
        .onTrue(ShootingCommands.decreaseShooterVelocityByPointTwo(subsystems));
    manipulatorController.back().onTrue(ShootingCommands.setShooterVelocityToSeven(subsystems));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autos.getAutonomous();
  }

  public void periodic() {
    operator.periodic();
  }
}
