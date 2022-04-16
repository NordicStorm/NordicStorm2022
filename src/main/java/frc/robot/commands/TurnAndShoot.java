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
    boolean actuallyShoot;
    boolean isStopper;
    static boolean inActualShootingMode = false; // this global variable allows us to stop other TurnAndShoot instances when the mode changes
    static boolean stopAllInstances = false;
    /**
     * Stop all shooting commands
     */
    public TurnAndShoot(boolean stop) {
        isStopper = stop;
    }
    /**
     * Take control of the drivetrain and barrel to take the shot
     * 
     * @param drivetrain
     * @param vision
     * @param timeout    in milliseconds
     * @param actuallyShoot
     */
    public TurnAndShoot(Drivetrain drivetrain, Barrel barrel, Vision vision, long timeout, boolean actuallyShoot) {
        this.drivetrain = drivetrain;
        this.barrel = barrel;
        this.vision = vision;
        this.timeout = timeout;
        this.manual = timeout == 30181;
        this.manualEnd = timeout == 30182;
        this.actuallyShoot = actuallyShoot;
        this.isStopper = false;
        //SmartDashboard.putNumber("sTilt", barrel.intakePos);
        //SmartDashboard.putNumber("sTop", 1000);
        //SmartDashboard.putNumber("sBottom", 1000);
        //SmartDashboard.putNumber("spin", 0.30);

        SmartDashboard.putBoolean("go", false);
        addRequirements(barrel);

    }
    

    @Override
    public void initialize() {
        if(isStopper){
            stopAllInstances = true;
        }else{
            stopAllInstances = false;
            this.endingTime = System.currentTimeMillis() + timeout;
            currentlyRunning = true;
            if(manual){
                barrel.autoAdjustRPM = false;
            }else{
                barrel.autoAdjustRPM = true;
            }
            shot = false;
            inActualShootingMode = actuallyShoot; // now if any others don't match this, they will end
        }
    }
    /**
     * Sets rotation power so we point toward the target (actually a little to the side to prevent bounce).
     * Returns true if the current angle is within tolerance
     */
    public boolean rotateTowardTarget() {
        double angleOffset = 0*Math.atan2(0.3, ShootingUtil.getCurrentDistance()); // aim 12in to the side
        double angleNeeded = ShootingUtil.getNeededTurnAngle();

        double angleDiff = Util.angleDiff(drivetrain.getGyroDegrees(), angleNeeded+angleOffset);
        double p = 0.12;
        if(vision.canSeeTarget){
            angleDiff = -vision.bestTarget.getYaw()-angleOffset;
        }
        double correction = angleDiff*p; // rotController.calculate(drivetrain.getGyroRadians(), angleNeeded);

        correction = Util.absClamp(correction, 5);
        drivetrain.setRotationSpeed(correction, 1);
        return Math.abs(angleDiff)<3;
    }

    double tilt = 76;
    double topRPM = 0;
    double bottomRPM = 0;
    boolean rotateGood = false;
    int timesRotGood = 0;
    boolean shot;
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
            if(actuallyShoot){
                barrel.setTiltAngle(tilt);
            }

            SmartDashboard.putNumber("sTilt", tilt);
            SmartDashboard.putNumber("sTop", topRPM);
            SmartDashboard.putNumber("sBottom", bottomRPM);
            SmartDashboard.putNumber("spin", topRPM/bottomRPM);
        }
        
        if(RobotContainer.leftJoystick.getRawButton(2)||!manual){
            rotateGood = rotateTowardTarget();
        }
        if(rotateGood){
            timesRotGood++;
        }else{
            timesRotGood = 0;
        }
        long timeLeft = endingTime - System.currentTimeMillis();
        //System.out.println(timesRotGood);
        boolean speedGood = PathUtil.linearSpeedFromChassisSpeeds(drivetrain.getSpeeds())<0.1;
        long shootTime = 300;
        if (actuallyShoot && ((barrel.readyToShoot() && (timesRotGood>3) && speedGood) || timeLeft<shootTime) && (RobotContainer.leftJoystick.getRawButton(6) || !manual) && !shot) {
            barrel.sendBothBallsUp();
            System.out.println("shot");
            shot = true;
            endingTime = System.currentTimeMillis() + shootTime;

            new KeepMovingTime(drivetrain, new ChassisSpeeds(0, 0, 0), shootTime).schedule(false);;
        }

    }

    @Override
    public boolean isFinished() {
        if(manualEnd){
            return !RobotContainer.leftJoystick.getRawButton(6);
        }
        if(RobotContainer.rightJoystick.getRawButton(2)){
            return true;
        }
        
        if (System.currentTimeMillis() > endingTime) {
            // cleanup here todo
            return true;
        }
        if(actuallyShoot != inActualShootingMode){
            return true; // this instance has become outdated
        }
        if(stopAllInstances){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        currentlyRunning = false;
    }
}
