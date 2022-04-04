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
        DrivetrainConfig config = drivetrain.getConfig().makeClone();
        config.maxVelocity = 4;
        config.maxAcceleration = 4;
        config.maxCentripetalAcceleration = 11;
        if(false){
            drivetrain.setPose(7.1882+halfWidth, 1.343025+halfWidth, 0);
            drivetrain.setAngleOffset(-90);

            MultiPartPath pathA;

            pathA = new MultiPartPath(drivetrain, config, null);
            pathA.addSequentialCommand(new FullStopPiece(pathA, 1));//ENDPOS:7.717,1.874
            pathA.setHeading(-90);
            pathA.addWaypoint(7.669, 1.514);
            pathA.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 3, drivetrain.myBallColor, 1));//ENDPOS:7.621,0.220
            pathA.setHeading(-120);
            pathA.addWaypoint(7.130, 1.538);
            pathA.stop();
            pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000));//ENDPOS:7.034,1.562
            pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000));//ENDPOS:6.914,1.514
            
            pathA.setHeading(170);
            pathA.addWaypoint(6.555, 1.538);
            pathA.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 3, drivetrain.myBallColor, 2));//ENDPOS:4.734,2.053
            //pathA.stop();
            //pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000));//ENDPOS:4.507,2.113
            pathA.setHeading(-135);
            
            pathA.addWaypoint(2.140, 2.294);
            pathA.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 2, drivetrain.myBallColor, 1));//ENDPOS:1.128,1.143
            pathA.addWaypoint(2.290, 1.718);
            pathA.addWaypoint(4.842, 1.454);
            pathA.addWaypoint(6.348, 1.926);
            pathA.stop();
            pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000));//ENDPOS:6.351,2.041
            pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000));//ENDPOS:6.471,2.101
    
            pathA.stop();
            
            addCommands(pathA.finalizePath());
        }
        if(true){
            drivetrain.setPose(5.74,4.41517, 0);
            drivetrain.setAngleOffset(-180);

            MultiPartPath pathB;
            pathB = new MultiPartPath(drivetrain, config, null);
            pathB.addSequentialCommand(new FullStopPiece(pathB, 1));//ENDPOS:5.950,4.578
            pathB.setHeading(-180);
            pathB.addWaypoint(6.232, 6.126);
            pathB.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 3, drivetrain.myBallColor, 2));//ENDPOS:4.614,6.210
            pathB.addWaypoint(6.555, 4.868);
            pathB.stop();
            pathB.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000));//ENDPOS:6.267,4.569
            pathB.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000));//ENDPOS:6.148,4.581
            pathB.setHeading(-135);
            pathB.addWaypoint(2.793, 4.042);
            pathB.addWaypoint(2.374, 2.221);
            pathB.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 2, drivetrain.myBallColor, 1));//ENDPOS:1.284,1.215
            pathB.setHeading(-180);
            pathB.addWaypoint(2.530, 2.245);
            pathB.addWaypoint(3.512, 3.754);
            pathB.addWaypoint(5.860, 4.162);
            pathB.stop();
            pathB.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000));//ENDPOS:6.315,4.365

            addCommands(pathB.finalizePath());

        }
       


    }

    
}