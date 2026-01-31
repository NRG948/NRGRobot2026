/*
 * Copyright (c) 2026 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot;

import com.nrg948.autonomous.Autonomous;
import com.nrg948.dashboard.annotations.DashboardBooleanBox;
import com.nrg948.dashboard.annotations.DashboardComboBoxChooser;
import com.nrg948.dashboard.annotations.DashboardDefinition;
import com.nrg948.dashboard.annotations.DashboardField;
import com.nrg948.dashboard.annotations.DashboardRadialGauge;
import com.nrg948.dashboard.model.GameField;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;

@DashboardDefinition
public final class RobotOperator {

  private Intake intake;
  private static Swerve drive;

  public RobotOperator(Subsystems subsystems) {
    autoChooser = Autonomous.getChooser(subsystems, "frc.robot");
    intake = subsystems.intake;
    drive = subsystems.drivetrain;
  }

  private static final double minSpeed = 0.0;
  private static final double maxSpeed = 5.0;

  /**
   * Returns magnitude of speed, which is calculated by the hypotenuse of the horizontal and vertical components of velocity. 
   */
  @DashboardRadialGauge(
      title = "Speed (M/S)",
      column = 5,
      row = 3,
      width = 2,
      height = 2,
      min = minSpeed,
      max = maxSpeed)
  private double velocity() {
    return Math.hypot(
        drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);
  }

  // TODO: Determine correct minimum velocity for if robot is intaking.
  @DashboardBooleanBox(title = "Intaking", column = 0, row = 0, width = 1, height = 1)
  private boolean intaking() {
    return intake.getCurrentVelocity() > 1.0;
  }

  // TODO: Determine correct minimum velocity for if robot is outtaking.
  @DashboardBooleanBox(title = "Outtaking", column = 1, row = 0, width = 1, height = 1)
  private boolean outtaking() {
    return intake.getCurrentVelocity() < -1.0;
  }

  // TODO: Implement logic for if robot is aligned and state it here.
  @DashboardBooleanBox(title = "Aligned", column = 5, row = 1, width = 2, height = 2)
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

  // TODO: Update pose every tick
  @DashboardComboBoxChooser(title = "Auto Routine", column = 0, row = 3, width = 2, height = 1)
  private final SendableChooser<Command> autoChooser;

  /*@DashboardMatchTime(title = "Match Time", row = 1, column = 7, width = 3, height = 1)
  public static double getMatchTime() {
    return MatchTime.getMatchTime();
  }*/
}
