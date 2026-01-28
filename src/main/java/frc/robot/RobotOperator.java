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
import com.nrg948.dashboard.model.GameField;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Subsystems;

@DashboardDefinition
public final class RobotOperator {
  public RobotOperator(Subsystems subsystems) {
    autoChooser = Autonomous.getChooser(subsystems, "frc.robot");
  }

  // TODO: Change this to be reflect beam break values
  @DashboardBooleanBox(title = "Intook", column = 0)
  private boolean intook = true;

  @DashboardBooleanBox(title = "ShooterCorrectRPM", column = 2)
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
  @DashboardComboBoxChooser(title = "Auto Routine", column = 8, row = 3, width = 2, height = 1)
  private final SendableChooser<Command> autoChooser;
}
