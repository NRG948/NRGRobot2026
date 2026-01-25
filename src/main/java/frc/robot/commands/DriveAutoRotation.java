/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.nrg948.dashboard.annotations.DashboardPIDController;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferences.DoubleValue;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Swerve;
import frc.robot.util.FieldUtils;

import com.nrg948.preferences.PIDControllerPreference;

/** A command that enables the driver to drive the robot using an Xbox controller. */
public class DriveAutoRotation extends Command {

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue RIGHT_TRIGGER_SCALAR =
      new RobotPreferences.DoubleValue("Drive", "Right Trigger Scalar", 0.25);

  private static final double RUMBLE_MIN_G = 1.0;
  private static final double RUMBLE_MAX_G = 8.0;

  private static final double DEADBAND = 0.08;

  private final Swerve drivetrain;
  private final CommandXboxController xboxController;

  private Pose2d hubAprilTag = FieldUtils.getHubAprilTag();
  private Pose2d hubLocation = new Pose2d(hubAprilTag.getMeasureX(), hubAprilTag.getMeasureY(), Rotation2d.kZero);
  
  private Pose2d currPose;
  private double rTarget; //in degrees
  

  //PID
  @DashboardPIDController(title = "Controller PID", column = 6, row = 0, width = 2, height = 3)
  private final PIDControllerPreference rotationPIDController =
      new PIDControllerPreference("Shooter", "Rotation PID Controller", 1, 0, 0);



  /** Creates a command which allows a human to drive the robot using an Xbox controller. */
  public DriveAutoRotation (Swerve drivetrain, CommandXboxController xboxController) {
    this.drivetrain = drivetrain;
    this.xboxController = xboxController;
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currPose = drivetrain.getPosition();

    double rSpeed;
    double xSpeed = -xboxController.getLeftY();
    double ySpeed = -xboxController.getLeftX();

    // The `powerScalar` linearly scales the robot's drive power from 1.0 (when the right trigger is
    // not pressed) down to RIGHT_TRIGGER_SCALAR (when the right trigger is fully depressed).
    double powerScalar =
        (RIGHT_TRIGGER_SCALAR.getValue() - 1.0) * xboxController.getRightTriggerAxis() + 1.0;

    double currentAngleRadians = currPose.getRotation().getRadians();
    double feedback = rotationPIDController.calculate(currentAngleRadians, getRotationtoHub());

    rSpeed =   feedback + (rotationPIDController.getSetpoint().velocity
                  / Swerve.getRotationalConstraints().maxVelocity);
                  
    // Applies deadbands to the x and y then multiplies all speeds by
    // the powerScalar, which allows finer driving control.
    xSpeed = MathUtil.applyDeadband(xSpeed, DEADBAND) * powerScalar;
    ySpeed = MathUtil.applyDeadband(ySpeed, DEADBAND) * powerScalar;

    drivetrain.drive(xSpeed, ySpeed, rSpeed, true);

    if (Swerve.ENABLE_RUMBLE.getValue()) {
      // Rumbles the driver controller based on a exponential scale based on acceleration between
      // min and max.
      double rumblePower =
          MathUtil.inverseInterpolate(RUMBLE_MIN_G, RUMBLE_MAX_G, drivetrain.getAcceleration());
      xboxController.setRumble(RumbleType.kBothRumble, rumblePower * rumblePower);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * 
   * @return Radians of angle difference from bot to hub
   */
  private double getRotationtoHub() {
    Rotation2d angleDiff = currPose.getTranslation().minus(hubLocation.getTranslation()).getAngle();
    double angleDiffRad = angleDiff.getRadians();
    return angleDiffRad;
  }

  private double calculateRotationSpeed(PIDController controller, double currentOrientation, double targetOrientation) {
  double feedback =
          controller.calculate(currentOrientation, targetOrientation);

      double rSpeed = feedback + (controller.getSetpoint().velocity / Swerve.getRotationalConstraints().maxVelocity);
      return rSpeed;
  }
 
}
