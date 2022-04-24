package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        //drivetrain.resetSwerve();

        boolean is4Ball = SmartDashboard.getBoolean("Is4Ball?", true);
        boolean doAnything = SmartDashboard.getBoolean("DoAuto?", true);
        boolean doLastBall = SmartDashboard.getBoolean("DoLastBall?", true);
        boolean singleBall = SmartDashboard.getBoolean("SingleBall?", false);
        if(!doAnything){
            return;
        }
        //vision.resetCam();
        double halfWidth = 0.46355;
        barrel.setTiltAngle(barrel.intakePos);
        drivetrain.resetAngle();
        DrivetrainConfig config = drivetrain.getConfig().makeClone();
        config.maxVelocity = 4;
        config.maxAcceleration = 4;
        config.maxCentripetalAcceleration = 11;
        if(is4Ball){
            config.maxVelocity = 4;
            config.maxAcceleration = 4;
            drivetrain.setPose(7.1882+halfWidth, 1.343025+halfWidth, 0);
            drivetrain.setAngleOffset(-90);

            MultiPartPath pathA;

            pathA = new MultiPartPath(drivetrain, config, null);
            pathA.addSequentialCommand(new FullStopPiece(pathA, 1));//ENDPOS:7.717,1.874
            pathA.setHeading(-90);
            pathA.addParallelCommand(new DebugLights(vision, 8000));

            pathA.addWaypoint(7.669, 1.514);
            pathA.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 3, drivetrain.myBallColor, 1, 600).withTimeout(5), 1);//ENDPOS:7.621,0.220
            pathA.addParallelCommand(new TurnAndShoot(drivetrain, barrel, vision, 99000, false, false));
            pathA.setHeading(-120);

            pathA.addWaypoint(7.130, 1.538);
            pathA.stop();
            pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:7.034,1.562
            pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:6.914,1.514
            if(doLastBall){
                pathA.setHeading(170);
                pathA.addWaypoint(6.555, 1.538);
                pathA.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 3, drivetrain.myBallColor, 2, 300));//ENDPOS:4.734,2.053
                //pathA.stop();
    
                pathA.setHeading(-135);
                
                pathA.addWaypoint(2.140, 2.294);
                pathA.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 2, drivetrain.myBallColor, 1, 600));//ENDPOS:1.128,1.143
                pathA.addParallelCommand(new TurnAndShoot(drivetrain, barrel, vision, 99000, false, false));
                pathA.addWaypoint(2.290, 1.718);
                pathA.addWaypoint(4.842, 1.454);
                pathA.addWaypoint(6.348, 1.926);
                pathA.stop();
                pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:6.351,2.041
                pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:6.471,2.101
        
            }
            
            pathA.stop();
            
            addCommands(pathA.finalizePath());
        } else if (singleBall) {
            drivetrain.setPose(6.232,3.790, 0);
            drivetrain.setAngleOffset(-180);

            MultiPartPath pathC;
            pathC = new MultiPartPath(drivetrain, config, null);
            pathC.setHeading(180);

            pathC.addSequentialCommand(new FullStopPiece(pathC, 1));//ENDPOS:6.232,3.790
            pathC.addWaypoint(4.351, 3.766);
            pathC.stop();
            pathC.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:3.872,3.742
            pathC.stop();
            addCommands(pathC.finalizePath());
        }
        else {
            drivetrain.setPose(5.74,4.41517, 0);
            drivetrain.setAngleOffset(-180);

            MultiPartPath pathB;
            pathB = new MultiPartPath(drivetrain, config, null);
            pathB.addSequentialCommand(new FullStopPiece(pathB, 1));//ENDPOS:5.950,4.578
            pathB.setHeading(-180);
            
            pathB.addWaypoint(6.232, 6.126);
            pathB.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 3, drivetrain.myBallColor, 2, 300).withTimeout(5), 1);//ENDPOS:4.614,6.210
            pathB.addParallelCommand(new TurnAndShoot(drivetrain, barrel, vision, 99000, false, false));
            pathB.addWaypoint(5.597, 4.904);
            pathB.stop();
            pathB.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:5.393,4.653
            pathB.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:5.297,4.593
            pathB.addWaypoint(4.171, 4.473);
            if(doLastBall){
                pathB.setHeading(-135);
                pathB.addWaypoint(2.793, 4.042);
                pathB.addWaypoint(0.697, 2.880);
                pathB.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 2, drivetrain.myBallColor, 1, 600));//ENDPOS:0.278,1.921
                pathB.addParallelCommand(new TurnAndShoot(drivetrain, barrel, vision, 99000, false, false));
                pathB.setHeading(-180);
                pathB.addWaypoint(1.619, 3.946);
                pathB.addWaypoint(3.896, 4.245);
                pathB.addWaypoint(5.309, 4.437);
                pathB.stop();
                pathB.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:5.549,4.413
            }
            
            pathB.stop();
            addCommands(pathB.finalizePath());

        }
       


    }

    
}