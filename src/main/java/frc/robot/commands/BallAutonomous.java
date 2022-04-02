package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.paths.MultiPartPath;
import frc.robot.subsystems.Barrel;
import frc.robot.subsystems.Drivetrain;

public class BallAutonomous extends AutoWithInit{

    Drivetrain drivetrain;
    Barrel barrel;
    public BallAutonomous(Drivetrain drivetrain, Barrel barrel){
        this.drivetrain = drivetrain;
    }
    @Override
    public void initializeCommands() {
        MultiPartPath path;
        path = new MultiPartPath(drivetrain);
        path.resetPosition(0, 0);
        path.addWaypoint(1, 1);
        path.addWaypoint(5, 5);
        path.addCommand(new FollowBall(drivetrain, barrel, true, true, 2, drivetrain.myBallColor));
        

        addCommands(path.finalizePath());
    }

    
}
