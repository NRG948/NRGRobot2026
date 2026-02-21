/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot;

import com.nrg948.dashboard.annotations.DashboardBooleanBox;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.nrg948.dashboard.annotations.DashboardField;
import com.nrg948.dashboard.annotations.DashboardMatchTime;
import com.nrg948.dashboard.annotations.DashboardRadialGauge;
import com.nrg948.dashboard.model.GameField;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.AprilTag;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.MatchTime;
import java.util.Optional;

@DashboardDefinition
public final class RobotOperator {

  private final Intake intake;
  private final Swerve drive;
  public final Optional<AprilTag> frontLeftCamera;
  public final Optional<AprilTag> frontRightCamera;
  public final Optional<AprilTag> backLeftCamera;
  public final Optional<AprilTag> backRightCamera;

  public RobotOperator(Subsystems subsystems) {
    intake = subsystems.intake;
    drive = subsystems.drivetrain;
    frontLeftCamera = subsystems.frontLeftCamera;
    frontRightCamera = subsystems.frontRightCamera;
    backLeftCamera = subsystems.backLeftCamera;
    backRightCamera = subsystems.backRightCamera;
  }

  private static final double MIN_SPEED = 0.0;
  private static final double MAX_SPEED = 5.0;

  /**
   * Returns magnitude of speed, which is calculated by the hypotenuse of the horizontal and
   * vertical components of velocity.
   */
  @DashboardRadialGauge(
      title = "Speed (M/S)",
      column = 4,
      row = 2,
      width = 2,
      height = 2,
      min = MIN_SPEED,
      max = MAX_SPEED)
  public double velocity() {
    return Math.hypot(
        drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);
  }

  // TODO: Determine correct minimum velocity for if robot is intaking.
  @DashboardBooleanBox(title = "Intaking", column = 0, row = 0, width = 1, height = 1)
  public boolean intaking() {
    return Math.signum(intake.getGoalVelocity()) > 0.0;
  }

  // TODO: Determine correct minimum velocity for if robot is outtaking.
  @DashboardBooleanBox(title = "Outtaking", column = 1, row = 0, width = 1, height = 1)
  public boolean outtaking() {
    return Math.signum(intake.getGoalVelocity()) < 0.0;
  }

  // TODO: Implement logic for if robot is aligned and state it here.
  @DashboardBooleanBox(title = "Aligned", column = 4, row = 0, width = 2, height = 2)
  private boolean isAligned = false;

  // TODO: Implement logic for correct RPM and state it here.
  @DashboardBooleanBox(title = "ShooterCorrectRPM", column = 2, row = 0, width = 2, height = 1)
  private boolean shooterCorrectRPM = true;

  @DashboardField(
      title = "Field",
      row = 1,
      column = 0,
      height = 2,
      width = 4,
      game = GameField.REBUILT)
  private Field2d field = new Field2d();

  @DashboardMatchTime(title = "Match Time", row = 0, column = 6, width = 3, height = 2)
  public static double getMatchTime() {
    return MatchTime.getMatchTime();
  }

  @DashboardBooleanBox(
      title = "Back Left Camera Connected",
      row = 4,
      column = 6,
      width = 3,
      height = 2)
  public boolean backLeftCameraIsConnected() {
    return backLeftCamera.isPresent() ? backLeftCamera.get().isCameraConnected() : false;
  }

  @DashboardBooleanBox(
      title = "Back Right Camera Connected",
      row = 4,
      column = 9,
      width = 3,
      height = 2)
  public boolean backRightCameraIsConnected() {
    return backRightCamera.isPresent() ? backRightCamera.get().isCameraConnected() : false;
  }

  public void periodic() {
    field.setRobotPose(drive.getPosition());
  }
}
