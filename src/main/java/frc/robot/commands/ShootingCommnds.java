package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Indexer;

public final class ShootingCommnds {
    
    // TODO: Implement method that fetches goal velocity for indexer. 
    public static Command shoot(Subsystems subsystem) {
        Indexer indexer = subsystem.indexer;
        return Commands.runOnce(() -> indexer.setGoalVelocity(1.0) , indexer);
    }

}
