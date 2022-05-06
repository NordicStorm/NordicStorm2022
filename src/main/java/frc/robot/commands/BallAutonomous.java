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
        boolean extendToFive = SmartDashboard.getBoolean("ExtendToFive?", false);

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
        config.maxAngularAcceleration = 8;
        config.maxAnglularVelocity = 12;

        if(is4Ball){ //path on
            config.maxVelocity = 4;
            config.maxAcceleration = 4;
            drivetrain.setPose(7.1882+halfWidth, 1.343025+halfWidth, 0);
            drivetrain.setAngleOffset(-90);

            MultiPartPath pathA;

            pathA = new MultiPartPath(drivetrain, config, null);
            pathA.addSequentialCommand(new FullStopPiece(pathA, 1));//ENDPOS:7.741,2.029
            pathA.setHeading(-90);
            pathA.addWaypoint(7.669, 1.514);

            pathA.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 3, drivetrain.myBallColor, 1, 100).withTimeout(5), 1);//ENDPOS:7.621,0.220
            //pathA.addParallelCommand(new DebugLights(vision, 8000, 1000));
            //pathA.addParallelCommand(new TurnAndShoot(drivetrain, barrel, vision, 99000, false, false));
            //pathA.addSequentialCommand(new DebugLights(vision, 8000, 200));

            pathA.setHeading(-120);

            pathA.addWaypoint(7.130, 1.538);
            pathA.addStop();
            //pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:7.034,1.562
            pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:6.914,1.514
            if(doLastBall){ //path on

                pathA.setHeading(170);
                pathA.addWaypoint(6.351, 1.562);
                pathA.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 3, drivetrain.myBallColor, 2, 100));//ENDPOS:4.734,2.053
                
                if(!extendToFive){//path on

                    pathA.setHeading(-135);
                    pathA.addWaypoint(2.140, 2.294);
                    pathA.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 2, drivetrain.myBallColor, 1, 200));//ENDPOS:1.128,1.143
                    //pathA.addParallelCommand(new TurnAndShoot(drivetrain, barrel, vision, 99000, false, false));
                    pathA.addWaypoint(2.290, 1.718);
                }
                
                pathA.addWaypoint(5.585, 1.921);
                pathA.addStop();
                pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:5.680,1.969
                if(extendToFive){ //path off
                    pathA.setHeading(-135);
                    pathA.addWaypoint(2.140, 2.294);
                    pathA.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 2, drivetrain.myBallColor, 1, 100));//ENDPOS:1.128,1.143
                    pathA.addWaypoint(2.254, 1.874);
                    pathA.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 1, drivetrain.myBallColor, 1, 200));//ENDPOS:1.667,1.023
                    pathA.addWaypoint(3.368, 1.287);
                    pathA.addWaypoint(6.327, 1.957);
                    pathA.addStop();
                    pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:6.555,2.005
                }
                //pathA.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:6.471,2.101
        
            }
            
            pathA.addStop();
            
            addCommands(pathA.finalizePath());
        } else if (singleBall) { //path on
            config.maxCentripetalAcceleration = 5;
            config.maxAcceleration = 3;

            drivetrain.setPose(6.232,3.790, 0);
            drivetrain.setAngleOffset(-180);

            MultiPartPath pathC;
            pathC = new MultiPartPath(drivetrain, config, null);
            pathC.setHeading(180);

            pathC.addSequentialCommand(new FullStopPiece(pathC, 1));//ENDPOS:6.232,3.790
            if(doLastBall){//path on
                pathC.addWaypoint(4.003, 4.305);
                pathC.setHeading(-135);
                pathC.addWaypoint(2.793, 4.042);
                pathC.addWaypoint(1.320, 2.796);
                pathC.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 1, drivetrain.myBallColor, 1, 200).withTimeout(3), 1);//ENDPOS:0.278,1.921
                pathC.setHeading(-180);

                pathC.addWaypoint(1.380, 2.988);
                //pathB.addParallelCommand(new TurnAndShoot(drivetrain, barrel, vision, 99000, false, false));
                pathC.addWaypoint(2.266, 3.814);
                pathC.addWaypoint(3.896, 4.245);
                pathC.addWaypoint(5.153, 4.030);
            }else{//path off
                pathC.addWaypoint(4.375, 4.042);
                pathC.addWaypoint(5.285, 4.078);
            }
            
            pathC.addStop();
            pathC.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:5.513,4.054
            pathC.addStop();
            addCommands(pathC.finalizePath());
        }
        else { //path on
            drivetrain.setPose(5.74,4.41517, 0);
            drivetrain.setAngleOffset(-180);

            MultiPartPath pathB;
            pathB = new MultiPartPath(drivetrain, config, null);
            pathB.addSequentialCommand(new FullStopPiece(pathB, 1));//ENDPOS:5.950,4.578
            if(doLastBall){//path on
                pathB.setHeading(-135);
                pathB.addWaypoint(2.793, 4.042);
                pathB.addWaypoint(1.320, 2.796);
                pathB.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 1, drivetrain.myBallColor, 1, 200).withTimeout(3), 1);//ENDPOS:0.278,1.921
                pathB.setHeading(-180);

                pathB.addWaypoint(1.380, 2.988);
                //pathB.addParallelCommand(new TurnAndShoot(drivetrain, barrel, vision, 99000, false, false));
                pathB.addWaypoint(2.266, 3.814);
                pathB.addWaypoint(3.896, 4.245);
                pathB.addWaypoint(5.309, 4.713);
                pathB.addStop();
                pathB.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:5.465,4.701
                pathB.addWaypoint(6.052, 5.359);
            }
            pathB.setHeading(-180);
            pathB.addWaypoint(6.232, 6.126);
            pathB.addSequentialCommand(new FollowBall(drivetrain, barrel, true, true, 3, drivetrain.myBallColor, 2, 100).withTimeout(5), 1);//ENDPOS:4.614,6.210
            //pathB.addParallelCommand(new TurnAndShoot(drivetrain, barrel, vision, 99000, false, false));
            pathB.addWaypoint(5.261, 4.940);
            pathB.addStop();
            pathB.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:5.345,4.940
            //pathB.addSequentialCommand(new TurnAndShoot(drivetrain, barrel, vision, 2000, true, true));//ENDPOS:5.297,4.593
            
            
            pathB.addStop();
            addCommands(pathB.finalizePath());

        }
       


    }

    
}