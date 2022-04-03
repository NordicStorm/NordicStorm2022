package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.paths.DrivetrainConfig;
import frc.robot.commands.paths.FullStopPiece;
import frc.robot.commands.paths.MultiPartPath;
import frc.robot.subsystems.Barrel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class BallAutonomous extends AutoWithInit{

    Drivetrain drivetrain;
    Barrel barrel;
    Vision vision;
    public BallAutonomous(Drivetrain drivetrain, Barrel barrel, Vision vision){
        this.drivetrain = drivetrain;
        this.barrel = barrel;
        this.vision = vision;
    }
    @Override
    public void initializeCommands() {
// !PATHWEAVER_INFO: {"trackWidth":0.9271,"gameName":"Rapid React","outputDir":"C:\\Users\\Nordic Storm 3018\\FRC\\NordicStorm2022\\src\\main\\java\\frc\\robot\\commands\\BallAutonomous.java"}
        //36.5 inch / 2 = 0.46355 m
        double halfWidth = 0.46355;
        MultiPartPath path;
        barrel.setTiltAngle(barrel.intakePos);
        drivetrain.resetAngle();
        drivetrain.setAngleOffset(-90);
        drivetrain.setPose(7.1882+halfWidth, 1.343025+halfWidth, 0);
        DrivetrainConfig config = drivetrain.getConfig();
        config.maxVelocity = 2;
        config.maxAcceleration = 1;

        path = new MultiPartPath(drivetrain, config, null);
        path.addSequentialCommand(new FullStopPiece(path, 1));//ENDPOS:7.652,1.807
        path.setHeading(-90);
        path.addWaypoint(7.669, 1.514);
        path.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 0.5, drivetrain.myBallColor, 0.5));//ENDPOS:7.621,0.220
        path.setHeading(180);

        path.addWaypoint(7.657, 0.759);
        path.stop();
        path.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000));//ENDPOS:7.621,0.747
        path.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000));//ENDPOS:7.549,0.759
        
        path.setHeading(180);
        path.addWaypoint(6.052, 1.634);
        path.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 1, drivetrain.myBallColor, 1));//ENDPOS:4.734,2.053
        path.setHeading(-135);
        
        path.addWaypoint(1.392, 1.838);
        path.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 0.5, drivetrain.myBallColor, 0.5));//ENDPOS:1.128,1.143
        path.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000));//ENDPOS:1.152,1.239
        path.stop(000);
        
        

        addCommands(path.finalizePath());
    }

    
}