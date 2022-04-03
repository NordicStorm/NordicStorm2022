// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.commands.paths.DrivetrainConfig;
import frc.robot.subsystems.Barrel;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OperatorControl extends CommandBase {

    private final Drivetrain drivetrain;
    private final Vision vision;
    private final Barrel barrel;
    private final Climbers climbers;
    private DrivetrainConfig config;

    public OperatorControl(Drivetrain drivetrain, Barrel barrel, Climbers climbers, Vision vision) {
        this.drivetrain = drivetrain;
        this.config = drivetrain.getConfig();
        this.barrel = barrel;
        this.vision = vision;
        this.climbers = climbers;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var leftStick = RobotContainer.leftJoystick;
        var rightStick = RobotContainer.rightJoystick;
        double forward = -rightStick.getY();
        double sideways = -rightStick.getX();
        double rot = -rightStick.getTwist();
        double throttle = rightStick.getThrottle();
        throttle=Util.map(throttle, 1, -1, 0.1, 1);
        if(forward<0.008 && rot>=0.14){//weird thing with joystick
            //rot = Util.map(rot, 0.15, in_max, out_min, out_max);
            rot -= 0.14;
        }
        if(forward>0.7 && false){
            drivetrain.driveVolts(new ChassisSpeeds(12, 0, 0));
            return;
        }
        throttle*=config.maxVelocity;
        

        forward = Util.applyDeadzone(forward, 0.1) * throttle;
        sideways = Util.applyDeadzone(sideways, 0.1) * throttle;
        rot = Util.applyDeadzone(rot, 0.2);
        rot=Util.signedSquare(rot);
        rot*=5;
        
        ChassisSpeeds localSpeeds = Util.rotateSpeeds(new ChassisSpeeds(forward, sideways, rot), drivetrain.getGyroRadians());
        
        
        drivetrain.limitDrive(localSpeeds, 0);
        SmartDashboard.putNumber("comX", localSpeeds.vxMetersPerSecond);
        barrel.setIntake(leftStick.getRawButton(4));

        if(ShootingUtil.getTimeToReady()<750){
            new TurnAndShoot(drivetrain, barrel, vision, 1000).schedule();
        }
    }
}
