package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.paths.MultiPartPath;
import frc.robot.subsystems.Drivetrain;

public class BallAutonomous extends AutoWithInit{

    Drivetrain drivetrain;
    public BallAutonomous(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
    }
    @Override
    public void initializeCommands() {
        MultiPartPath path;
        path = new MultiPartPath(drivetrain);
        

        addCommands(path.finalizePath());
    }

    
}
