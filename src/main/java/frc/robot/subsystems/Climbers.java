// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Util;

public class Climbers extends SubsystemBase {

    private Drivetrain drivetrain;
    private Vision vision;
    private Barrel barrel;

    private CANSparkMax leftMotor = new CANSparkMax(14, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(18, MotorType.kBrushless);
    private SparkMaxPIDController leftPID = leftMotor.getPIDController();
    private SparkMaxPIDController rightPID = rightMotor.getPIDController();
    private ProfiledPIDController leftProfiledPID;
    private ProfiledPIDController rightProfiledPID;

    private RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightMotor.getEncoder();

    int pidSlot = 0; // 0 is for pulling up, 1 is for extending with no load
    public Climbers() {

        configureClimberSpark(leftMotor, false);
        configureClimberSpark(rightMotor, true);
        
    }
    public void setOtherSubsystems(Drivetrain drivetrain, Barrel barrel, Vision vision){
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.barrel = barrel;
        
    }

    //Start down position = 0
    //Max up = 51
    @Override
    public void periodic() {
        //double velo = leftEncoder.getVelocity();
        //checkResetPos(leftEncoder, leftLimitSwitch);
        //checkResetPos(rightEncoder, rightLimitSwitch);

        
        //leftPID.setReference(pos, CANSparkMax.ControlType.kPosition, pidSlot);
        //SmartDashboard.putNumber("currentLeft", leftMotor.getOutputCurrent());

        //SmartDashboard.putNumber("temp", leftMotor.getMotorTemperature());
        SmartDashboard.putNumber("leftPos", leftEncoder.getPosition());
        //SmartDashboard.putNumber("rightPos", rightEncoder.getPosition());
        
    }

    private void checkResetPos(RelativeEncoder encoder, DigitalInput limSwitch) {
        if(limSwitch.get()){
            if(Math.abs(encoder.getPosition())>0.5){
                encoder.setPosition(0);
            }
        }
    }
    private void configureClimberSpark(CANSparkMax spark, boolean reverse){
        SparkMaxPIDController pid = spark.getPIDController();
        spark.disableVoltageCompensation();
        spark.setInverted(reverse);
        int sign = reverse?-1:1;
        spark.setSoftLimit(SoftLimitDirection.kReverse, -49.5f);
        spark.setSoftLimit(SoftLimitDirection.kForward, -0.2f);
        spark.enableSoftLimit(SoftLimitDirection.kForward, true);
        spark.enableSoftLimit(SoftLimitDirection.kReverse, true);

        pid.setP(0, 0);
        pid.setI(0, 0);
        pid.setD(0, 0);

        pid.setP(0, 1);
        pid.setI(0, 1);
        pid.setD(0, 1);
        
        spark.setIdleMode(IdleMode.kBrake);
        
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 503); 


    }
    private ProfiledPIDController makeProfiledPid(){
        Constraints constraints = new Constraints(1, 1);
        ProfiledPIDController p = new ProfiledPIDController(0, 0, 0, constraints);
        return p;
    }
    /**
     * Positive means extending up
     * @param volts
     */
    public void setVoltages(double volts){
        leftPID.setReference(volts, CANSparkMax.ControlType.kVoltage);
        rightPID.setReference(volts, CANSparkMax.ControlType.kVoltage);
    }
    public void setRaw(double power){
        leftMotor.set(power);
        rightMotor.set(power);
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

    public double getLeftSpeed(){
        return leftEncoder.getVelocity();
    }
    public double getRightSpeed(){
        return rightEncoder.getVelocity();
    }

    public double getLeftPos(){
        return leftEncoder.getPosition();
    }
    public double getRightPos(){
        return rightEncoder.getPosition();
    }
}
