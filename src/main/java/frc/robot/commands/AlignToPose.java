// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.FieldUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToPose extends Command {

  private static final DataLog LOG = DataLogManager.getLog();
  private static final double MAX_TRANSLATIONAL_POWER = 0.30;
  private static final double MAX_ROTATIONAL_POWER = 0.5;

  // TODO: Change these to Robot 
  public static double Px = 1;
  public static double Py = 1;
  public static double Pr = 0.02;

  protected final Swerve drivetrain;

  protected Pose2d targetPose;

  private final PIDController xController = new PIDController(Px, 0, 0);
  private final PIDController yController = new PIDController(Py, 0, 0);
  private final PIDController rController = new PIDController(Pr, 0, 0);

  private final DoubleLogEntry xErrorLog = new DoubleLogEntry(LOG, "Vision/Pose X Error");
  private final DoubleLogEntry yErrorLog = new DoubleLogEntry(LOG, "Vision/Pose Y Error");
  private final DoubleLogEntry rErrorLog = new DoubleLogEntry(LOG, "Vision/Pose Yaw Error");

  private double xTarget;
  private double yTarget;
  private double rTarget; // in degrees

  private final StructLogEntry<Pose2d> poseTargetLog =
      StructLogEntry.create(LOG, "Vision/Pose Target", Pose2d.struct);

  protected AlignToPose(Subsystems subsystems) {
    this.drivetrain = subsystems.drivetrain;

    addRequirements(drivetrain);
  }

  protected AlignToPose(Subsystems subsystems, Pose2d targetPose) {
    this(subsystems);

    this.targetPose = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xTarget = targetPose.getX();
    yTarget = targetPose.getY();
    rTarget = targetPose.getRotation().getDegrees();

    poseTargetLog.append(targetPose);

    // Set up the PID controllers to drive to the target pose.
    xController.setSetpoint(xTarget);
    yController.setSetpoint(yTarget);
    rController.setSetpoint(rTarget);

    xController.setPID(Px, 0, 0);
    yController.setPID(Py, 0, 0);
    rController.setPID(Pr, 0, 0);

    xController.setTolerance(Constants.VisionConstants.POSE_ALIGNMENT_TOLERANCE_XY);
    yController.setTolerance(Constants.VisionConstants.POSE_ALIGNMENT_TOLERANCE_XY);
    rController.setTolerance(Constants.VisionConstants.POSE_ALIGNMENT_TOLERANCE_R);

    rController.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentRobotPose = drivetrain.getPosition();
    double currentX = currentRobotPose.getX();
    double currentY = currentRobotPose.getY();
    double currentR = currentRobotPose.getRotation().getDegrees();

    xErrorLog.append(xTarget - currentX);
    yErrorLog.append(yTarget - currentY);
    rErrorLog.append(rTarget - currentR);

    double xSpeed =
        MathUtil.clamp(
            xController.calculate(currentX), -MAX_TRANSLATIONAL_POWER, MAX_TRANSLATIONAL_POWER);
    double ySpeed =
        MathUtil.clamp(
            yController.calculate(currentY), -MAX_TRANSLATIONAL_POWER, MAX_TRANSLATIONAL_POWER);
    double rSpeed =
        MathUtil.clamp(
            rController.calculate(currentR), -MAX_ROTATIONAL_POWER, MAX_ROTATIONAL_POWER);

    if (FieldUtils.isRedAlliance()) {
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
    }

    drivetrain.drive(xSpeed, ySpeed, rSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint();
  }
}
