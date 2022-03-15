// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Barrel extends SubsystemBase {
    private Drivetrain drivetrain;
    private Vision vision;
    private Climbers climbers;

    private CANSparkMax bottomStage = new CANSparkMax(10, MotorType.kBrushed);
    private CANSparkMax topStage = new CANSparkMax(11, MotorType.kBrushless);
    private RelativeEncoder topStageEncoder = topStage.getEncoder();
    private DigitalInput bottomSensor = new DigitalInput(1);
    private DigitalInput topSensor = new DigitalInput(2);

    public Barrel() {

    }

    public void setOtherSubsystems(Drivetrain drivetrain, Climbers climbers, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.climbers = climbers;
    }

    long bottomStopIntake = 0;
    long bottomStopHandoff = 0;
    long bottomStopFinalShift = 0;

    long bottomHandoffDuration = 500;
    long bottomHandoffDeadzone = bottomHandoffDuration - 200;
    long bottomShiftDuration = 100;

    long topStopIntake = 0;
    long topStopHandoff = 0;
    long topStopFinalShift = 0;

    long topIntakeDuration = 500;
    long topHandoffDuration = 500;
    long topHandoffDeadzone = topHandoffDuration - 200;
    long topShiftDuration = 100;


    @Override
    public void periodic() {
        updateIndexer();
        updateShooter();
    }

    private void updateIndexer() {
        boolean bottomBall = bottomSensor.get();
        boolean topBall = topSensor.get();
        double topVelo = topStageEncoder.getVelocity();
        boolean topIsOpen = (!topBall) && topVelo<5; // we can move a ball into top if there is not a ball there and it is not spinning fast
        long now = System.currentTimeMillis();
        boolean runBottom = false;
        double desTopSpeed = 0;

        if (topStopIntake > now) {
            desTopSpeed = 1;
            long timeLeft = topStopHandoff - now;
            if(timeLeft < topHandoffDeadzone && topBall){
                topStopIntake = 0;
                topStopFinalShift = now+topShiftDuration;
            }
        }
        if (topStopFinalShift > now) {
            desTopSpeed = 1;
        }
        if (topStopHandoff > now) {
            desTopSpeed = 9;
        }

        if (bottomStopHandoff > now) {
            runBottom = true;
        } else {
            if(bottomBall && topIsOpen){
                bottomStopHandoff = now + bottomHandoffDuration;
                topStopIntake = now + topIntakeDuration;
                runBottom = true;
            }
        }
        if (bottomStopIntake > now) {
            runBottom = true;
            long timeLeft = bottomStopHandoff - now;

            if (timeLeft < bottomHandoffDeadzone && bottomBall) {
                bottomStopIntake = 0;
                bottomStopFinalShift = now + bottomShiftDuration;
            }
        }
        if (bottomStopFinalShift > now) {
            runBottom = true;
        }
        

        if (runBottom) {
            bottomStage.set(1);
        }else{
            bottomStage.set(0);
        }
        topStage.set(desTopSpeed);

    }
    private void updateShooter() {

    }

    public void setIntake(boolean running){
        if(running){
            bottomStopIntake = System.currentTimeMillis() + 1000000;
        }else{
            bottomStopIntake = 0;

        }
    }
}
