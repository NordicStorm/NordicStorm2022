package frc.robot.commands;

import java.nio.file.Path;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.commands.paths.PathUtil;
import frc.robot.subsystems.Barrel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TurnAndShoot extends CommandBase implements CommandPathPiece{

    public static boolean currentlyRunning = false;
    private Drivetrain drivetrain;
    private Barrel barrel;
    private Vision vision;
    private long timeout;
    private long endingTime;

    boolean done = false;
    boolean manual = false;
    boolean manualEnd = false;
  
    /**
     * Take control of the drivetrain and barrel to take the shot
     * 
     * @param drivetrain
     * @param vision
     * @param timeout    in milliseconds
     */
    public TurnAndShoot(Drivetrain drivetrain, Barrel barrel, Vision vision, long timeout) {
        this.drivetrain = drivetrain;
        this.barrel = barrel;
        this.vision = vision;
        this.timeout = timeout;
        this.manual = timeout == 30181;
        this.manualEnd = timeout == 30182;
        //SmartDashboard.putNumber("sTilt", barrel.intakePos);
        //SmartDashboard.putNumber("sTop", 1000);
        //SmartDashboard.putNumber("sBottom", 1000);
        //SmartDashboard.putNumber("spin", 0.30);

        SmartDashboard.putBoolean("go", false);
        addRequirements(barrel);

    }
    

    @Override
    public void initialize() {
        this.endingTime = System.currentTimeMillis() + timeout;
        currentlyRunning = true;
        if(manual){
            barrel.autoAdjustRPM = false;
        }else{
            barrel.autoAdjustRPM = true;

        }
    }
    
    public void rotateTowardTarget() {
        
        double angleNeeded = ShootingUtil.getNeededTurnAngle();
        double angleDiff = Util.angleDiff(drivetrain.getGyroDegrees(), angleNeeded);
        double correction = angleDiff*0.16; // rotController.calculate(drivetrain.getGyroRadians(), angleNeeded);
        
        if(vision.canSeeTarget && ShootingUtil.getCurrentLinearSpeed()<0.1){
            angleDiff = -vision.bestTarget.getYaw();
            correction = angleDiff*0.16;
        }
        correction = Util.absClamp(correction, 10);
        drivetrain.setRotationSpeed(correction, 1);
        rotateGood = Math.abs(angleDiff)<3;
        
    }

    double tilt = 76;
    double topRPM = 0;
    double bottomRPM = 0;
    boolean rotateGood = false;
    int timesRotGood = 0;
    @Override
    public void execute() {

        double lastDistance = ShootingUtil.getCurrentDistance();
        if (SmartDashboard.getBoolean("go", false) && manual) {
            SmartDashboard.putBoolean("go", false);
            tilt = SmartDashboard.getNumber("sTilt", barrel.intakePos);
            bottomRPM = SmartDashboard.getNumber("sBottom", 1000);
            topRPM = bottomRPM*SmartDashboard.getNumber("spin", 0.6);
            
            SmartDashboard.putNumber("sTop", topRPM);
            //distance is all to lens!
            SmartDashboard.putString("csv", lastDistance+","+bottomRPM+","+topRPM+","+tilt); //175 fake angles
            barrel.setFlywheels(topRPM, bottomRPM);
            barrel.setTiltAngle(tilt);
        }else{
            
        }
        if(RobotContainer.leftJoystick.getRawButton(5) || !manual){
            topRPM = ShootingUtil.getShootingTopSpeed(lastDistance);
            bottomRPM = ShootingUtil.getShootingBottomSpeed(lastDistance);
            tilt = ShootingUtil.getShootingTilt(lastDistance);
            //topRPM = 5000;
            //barrel.setFlywheels(topRPM, bottomRPM);
            barrel.setTiltAngle(tilt);

            SmartDashboard.putNumber("sTilt", tilt);
            SmartDashboard.putNumber("sTop", topRPM);
            SmartDashboard.putNumber("sBottom", bottomRPM);
            SmartDashboard.putNumber("spin", topRPM/bottomRPM);
        }
        
        if(RobotContainer.leftJoystick.getRawButton(2)||!manual){
            rotateTowardTarget();
        }
        if(rotateGood){
            timesRotGood++;
        }else{
            timesRotGood = 0;
        }
        boolean speedGood = PathUtil.linearSpeedFromChassisSpeeds(drivetrain.getSpeeds())<0.1 || RobotContainer.leftJoystick.getRawButton(10);
        if (barrel.readyToShoot() && (rotateGood) && speedGood && (RobotContainer.leftJoystick.getRawButton(6) || !manual)) {
            barrel.shoot();
            System.out.println("shot");
            endingTime = System.currentTimeMillis() + 200;
            ChassisSpeeds localSpeeds = drivetrain.getSpeeds();

            new KeepMovingTime(drivetrain, Util.rotateSpeeds(localSpeeds, -drivetrain.getGyroRadians()), 200);
        }

    }

    @Override
    public boolean isFinished() {
        if(manualEnd){
            return !RobotContainer.leftJoystick.getRawButton(6);
        }
        if (System.currentTimeMillis() > endingTime) {
            // cleanup here todo
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        currentlyRunning = false;
    }
}
