// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util;

public class Climbers extends SubsystemBase {

    private Drivetrain drivetrain;
    private Vision vision;
    private Barrel barrel;

    private CANSparkMax leftMotor = new CANSparkMax(14, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(15, MotorType.kBrushless);
    private SparkMaxPIDController leftPID = leftMotor.getPIDController();
    private SparkMaxPIDController rightPID = rightMotor.getPIDController();
    private RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightMotor.getEncoder();

    int pidSlot = 0; // 0 is for pulling up, 1 is for extending with no load
    public Climbers() {

        configureClimberSpark(leftMotor);
        configureClimberSpark(rightMotor);
        
    }
    public void setOtherSubsystems(Drivetrain drivetrain, Barrel barrel, Vision vision){
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.barrel = barrel;
    }

    @Override
    public void periodic() {
        double velo = leftEncoder.getVelocity();
        setVoltages(Util.leftDebug()*12);
        
    }

    private void configureClimberSpark(CANSparkMax spark){
        SparkMaxPIDController pid = spark.getPIDController();
        
        pid.setP(0, 0);
        pid.setI(0, 0);
        pid.setD(0, 0);

        pid.setP(0, 1);
        pid.setI(0, 1);
        pid.setD(0, 1);
        
        spark.setIdleMode(IdleMode.kBrake);

    }

    public void setVoltages(double volts){
        leftPID.setReference(volts, CANSparkMax.ControlType.kVoltage);
        rightPID.setReference(volts, CANSparkMax.ControlType.kVoltage);
    }

    public void setPositions(double pos){
        leftPID.setReference(pos, CANSparkMax.ControlType.kPosition, pidSlot);
        rightPID.setReference(pos, CANSparkMax.ControlType.kPosition, pidSlot);
    }
    /**
     * Set which slot to use for the position control of the climbers
     * @param slot 0 is for pulling up, 1 is for extending with no load.
     */
    public void setPidSlot(int slot){

    }
}
