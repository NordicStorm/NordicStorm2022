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
        barrel.setTiltAngle(barrel.intakePos);
        drivetrain.resetAngle();
        drivetrain.setAngleOffset(-90);
        drivetrain.setPose(7.1882+halfWidth, 1.343025+halfWidth, 0);
        DrivetrainConfig config = drivetrain.getConfig().makeClone();
        config.maxVelocity = 3;
        config.maxAcceleration = 2;
        MultiPartPath pathA;

        pathA = new MultiPartPath(drivetrain, config, null);
        pathA.addSequentialCommand(new FullStopPiece(pathA, 1));//ENDPOS:7.652,1.807
        pathA.setHeading(-90);
        pathA.addWaypoint(7.669, 1.514);
        pathA.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 1, drivetrain.myBallColor, 1));//ENDPOS:7.621,0.220
        pathA.addWaypoint(7.657, 0.759);
        pathA.stop();
        pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000));//ENDPOS:7.621,0.747
        pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000));//ENDPOS:7.549,0.759
        
        pathA.setHeading(170);
        pathA.addWaypoint(6.052, 1.634);
        pathA.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 2, drivetrain.myBallColor, 2));//ENDPOS:4.734,2.053
        pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000));//ENDPOS:4.507,2.113
        pathA.setHeading(-135);
        
        pathA.addWaypoint(2.099, 2.425);
        pathA.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 1, drivetrain.myBallColor, 1));//ENDPOS:1.128,1.143
        pathA.addWaypoint(1.739, 1.909);
        pathA.stop();
        pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000));//ENDPOS:2.075,2.484

        pathA.setHeading(-90);
        pathA.stop(4000);
        pathA.addWaypoint(7.477, 1.874);
        pathA.stop();
        

        //addCommands(pathA.finalizePath());

        MultiPartPath pathB;

        pathB = new MultiPartPath(drivetrain, config, null);
        pathB.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000));//ENDPOS:7.652,1.807
        addCommands(pathB.finalizePath());

    }

    
}