// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.commands.RollingAverage;
import frc.robot.commands.ShootingUtil;

public class Barrel extends SubsystemBase {
    private Drivetrain drivetrain;
    private Vision vision;
    private Climbers climbers;

    private CANSparkMax bottomStage = new CANSparkMax(10, MotorType.kBrushless);
    private CANSparkMax topStage = new CANSparkMax(11, MotorType.kBrushless);
    
    private RelativeEncoder topStageEncoder = topStage.getEncoder();
    private DigitalInput bottomSensor = new DigitalInput(20);
    private DigitalInput topSensor = new DigitalInput(21);

    private CANSparkMax screw = new CANSparkMax(17, MotorType.kBrushless);
    private SparkMaxPIDController screwPID = screw.getPIDController();
    private SparkMaxAnalogSensor screwEncoder = screw.getAnalog(Mode.kAbsolute);

    private CANSparkMax topWheel = new CANSparkMax(12, MotorType.kBrushless);
    private SparkMaxPIDController topWheelPID = topWheel.getPIDController();
    private RelativeEncoder topWheelEncoder = topWheel.getEncoder();

    private CANSparkMax bottomWheel = new CANSparkMax(13, MotorType.kBrushless);
    private SparkMaxPIDController bottomWheelPID = bottomWheel.getPIDController();
    private RelativeEncoder bottomWheelEncoder = bottomWheel.getEncoder();


    public Barrel() {
        bottomStage.enableVoltageCompensation(12);
        bottomStage.setIdleMode(IdleMode.kBrake);
        topStage.enableVoltageCompensation(12);
        topStage.setIdleMode(IdleMode.kBrake);

        bottomStage.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 505);
        bottomStage.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 507);
        bottomStage.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 509);
        bottomStage.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 511);

        topStage.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 513);
        topStage.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 515);
        topStage.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 517);



        screwEncoder.setPositionConversionFactor(0.296*360);
        screwEncoder.setVelocityConversionFactor(0.296*360);
        screw.enableVoltageCompensation(12);
        screw.setIdleMode(IdleMode.kBrake);
        screwPID.setFeedbackDevice(screwEncoder);
        
        //screwPID.setP(25);
        //screwPID.setI(0);
        screw.setInverted(false);
        //screw.burnFlash();
        configureFlywheelMotor(topWheel);
        configureFlywheelMotor(bottomWheel);
        //SmartDashboard.putNumber("tiltOffset", angleOffset);
        
    }

    public void setOtherSubsystems(Drivetrain drivetrain, Climbers climbers, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.climbers = climbers;
    }

    long bottomStopIntake = 0;
    long bottomStopHandoff = 0;
    long bottomStopFinalShift = 0;

    long bottomHandoffDuration = 300;
    long bottomHandoffDeadzone = bottomHandoffDuration - 200;
    long bottomShiftDuration = 00;

    long topStopIntake = 0;
    long topStopHandoff = 0;
    long topStopFinalShift = 0;

    long topIntakeDuration = 500;
    long topHandoffDuration = 400;
    long topHandoffDeadzone = topHandoffDuration - 200;
    long topShiftDuration = 0;

    boolean topBallAvailable = false;

    @Override
    public void periodic() {
        updateIndexer();
        updateTilt();
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
            desTopSpeed = 0.5;
            long timeLeft = topStopHandoff - now;
            // if the handoff time left is passed the deadzone
            if(timeLeft < topHandoffDeadzone && topBall){
                topStopIntake = 0;
                topStopFinalShift = now+topShiftDuration;
            }
        }
        if (topStopFinalShift > now) {
            desTopSpeed = 0.75;
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
        if(RobotContainer.leftJoystick.getRawButton(11)){
            topStage.set(-1);
            bottomStage.set(-1);
            return;
        }
        if (runBottom) {
            bottomStage.set(0.5);
        }else{
            bottomStage.set(0);
        }
        topStage.set(desTopSpeed);
        topBallAvailable = desTopSpeed == 0 && topBall;
        SmartDashboard.putBoolean("bottomBall", bottomBall);
        SmartDashboard.putBoolean("topBall", topBall);

    }

    public void setIntake(boolean running){
        if(running){
            bottomStopIntake = System.currentTimeMillis() + 1000;
        }else{
            //bottomStopIntake = 0; // don't do because this way all intake finishes first

        }
    }
    /**
     * Is there a ball in the top stage that is still and loaded to fire?
     * @return
     */
    public boolean ballAvailableToShoot(){
        return topBallAvailable;
    }
    /**
     * Move the top stage so the ball enters the flywheels. Will do nothing if there is not ballAvailableToShoot()
     */
    private void sendBallToShooter(){
        //if(ballAvailableToShoot()){
        topStopHandoff = System.currentTimeMillis()+topHandoffDuration;
        //}else{
            
        //}
    }
    public void sendBothBallsUp(){
        
        topStopHandoff = System.currentTimeMillis()+topHandoffDuration*2;
        bottomStopHandoff = System.currentTimeMillis()+topHandoffDuration*2;

    }

    //tilt stuff
    public final double maxAngle = 75.0;
    public final double minAngle = 20;
    public final double intakePos = 71.5;
    public RollingAverage posAverage = new RollingAverage(5);
    double targetTilt = 0;
    double currentTiltAng = 0;
    public boolean autoBarrel = true;
    private void updateTilt() {
        //angleOffset = SmartDashboard.getNumber("tiltOffset", angleOffset);
        
        posAverage.put(screwEncoder.getPosition());
        
        currentTiltAng = getTiltAngleFromSensor();
        if(targetTilt == 0){
            targetTilt = currentTiltAng;
        }
        SmartDashboard.putNumber("tiltang", currentTiltAng);
        double x = RobotContainer.leftJoystick.getZ();

        //setTiltAngle(Util.map(x, -1, 1, minAngle, maxAngle));
        if(autoBarrel){
            if(Math.abs(currentTiltAng-targetTilt)<1){
                screw.set(0);
            }else{
                setTiltAngle(targetTilt);
            }
        }
        

        //intakeHeight=79.4 degrees
        
        if(Math.abs(x)>0.8){
            //screw.set(x*1);
        }else{
            //screw.set(0);
        }
   }
   private double angleOffset = 152.79; //(178-39.0);//36.5
   // increasing the offset moves the barrel down
    /**
     * 
     * @param angle the angle in degrees, where 90 would be straight up.
     * @return
     */
    public void setTiltAngle(double angle){
        if(Math.abs(angle-currentTiltAng)<0.5){
            return;
        }
        angle = Util.clamp(angle, minAngle, maxAngle);
        targetTilt = angle;
        double adj = ((angle+angleOffset));
        //System.out.println(adj);
        screwPID.setReference(adj, ControlType.kPosition);
    }
    /**
     * 
     * @return the current measured barrel angle in degrees, where 90 would be straight up.
     */
    private double getTiltAngleFromSensor(){
        
        return getRawTiltAngle() - 1*angleOffset;
    }

    private double getRawTiltAngle(){
        
        return posAverage.get();
    }
   /**
     * 
     * @return the current measured barrel angle in degrees, where 90 would be straight up.
     */
    public double getTiltAngle(){
        return currentTiltAng;
    }

    public void rawScrew(double power){
        screw.set(power);
    }
    
    //Shooter stuff
    double topTargetRPM = 0;
    double bottomTargetRPM = 0;
    double topCurrentRPM = 0;
    double bottomCurrentRPM = 0;
    public boolean autoAdjustRPM = true;
    private void updateShooter() {
        topCurrentRPM = topWheelEncoder.getVelocity();
        bottomCurrentRPM = bottomWheelEncoder.getVelocity();

        SmartDashboard.putNumber("topShooterVelo", topCurrentRPM);
        SmartDashboard.putNumber("bottomShooterVelo", bottomCurrentRPM);

        double distance = ShootingUtil.getCurrentDistance();
        double topRPM = ShootingUtil.getShootingTopSpeed(distance);
        double bottomRPM = ShootingUtil.getShootingBottomSpeed(distance);
        //topRPM = 5000;
        if(autoAdjustRPM){
            setFlywheels(topRPM, bottomRPM);
        }
        //setFlywheelsRaw(Util.leftDebug(), Util.leftDebug());
        
    }
    /**
     * Configures the coasting, PID gains, etc for one of the flywheels
     */
    private void configureFlywheelMotor(CANSparkMax motor){
        SparkMaxPIDController pid = motor.getPIDController();
        motor.setInverted(true);
        motor.enableVoltageCompensation(12);
        motor.setIdleMode(IdleMode.kCoast);
        //pid.setP(0); 0.0003699999942909926
        //pid.setI(0);
        //pid.setD(0);
        //pid.setFF(0);0.00019000006432179362
        
    }
    /**
     * Set the target rpm of the top and bottom flywheels. Both must be positive.
     */
    public void setFlywheels(double topRPM, double bottomRPM){
        if(topRPM<0 || bottomRPM<0){
            System.out.println("ERROR: Tried to set shooter RPM to negative");
            return;
        }
        topTargetRPM = topRPM;
        bottomTargetRPM = bottomRPM;
        topWheelPID.setReference(topRPM, CANSparkMax.ControlType.kVelocity);
        bottomWheelPID.setReference(bottomRPM, CANSparkMax.ControlType.kVelocity);
    }

    public void setFlywheelsRaw(double top, double bottom){
        if(top<0 || bottom<0){
            System.out.println("ERROR: Tried to set shooter power to negative");
            return;
        }
        topWheelPID.setReference(top, CANSparkMax.ControlType.kDutyCycle);
        bottomWheelPID.setReference(bottom, CANSparkMax.ControlType.kDutyCycle);
    }

    public double getTopRPM(){
        return topCurrentRPM;
    }
    public double getBottomRPM(){
        return bottomCurrentRPM;
    }
    int timesGood = 0;
    /**
     * If everything is ready to shoot: RPM, Tilt, and sensor
     * @return
     */
    public boolean readyToShoot(){
        //System.out.println("top"+topCurrentRPM);
        //System.out.println("bottom"+bottomCurrentRPM);
        //System.out.println("bottomtarget"+bottomTargetRPM);
        
        double tilt = getTiltAngle();
        //System.out.println("tilt"+tilt);

        if(Util.close(topCurrentRPM, topTargetRPM, 60) &&
           Util.close(bottomCurrentRPM, bottomTargetRPM, 60) &&
           Util.close(tilt, targetTilt, 1) &&
           ballAvailableToShoot()
        ){
            timesGood+=1;
        }else{
            timesGood = 0;
        }

        return timesGood>=5;
    }
    /**
     * Feed and actually shoot the ball
     */
    public void shoot(){
        
        sendBallToShooter();
        
    }

    public boolean hasBottomBall() {
        return bottomSensor.get();
    }

    public boolean hasTopBall(){
        return topSensor.get();

    }

    public void resetToPos(double pos) {
        angleOffset = getRawTiltAngle()-pos;
        SmartDashboard.putNumber("tiltOffset", angleOffset);
    }

    public double getTiltVelocity(){
        
        return screwEncoder.getVelocity();
    }
   

}
