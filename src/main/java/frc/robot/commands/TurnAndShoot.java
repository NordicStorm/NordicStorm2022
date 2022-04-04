package frc.robot.commands;

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

    private Drivetrain drivetrain;
    private Barrel barrel;
    private Vision vision;
    private long timeout;
    private long endingTime;

    boolean done = false;
    boolean manual = false;
  
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
        this.manual = timeout > 99999;
        SmartDashboard.putNumber("sTilt", barrel.intakePos);
        SmartDashboard.putNumber("sTop", 1000);
        SmartDashboard.putNumber("sBottom", 1000);
        SmartDashboard.putNumber("spin", 0.30);

        SmartDashboard.putBoolean("go", false);

    }

    @Override
    public void initialize() {
        this.endingTime = System.currentTimeMillis() + timeout;
    }
    
    public void rotateTowardTarget() {
        double angleNeeded = ShootingUtil.getNeededTurnAngle();
        double angleDiff = Util.angleDiff(drivetrain.getGyroDegrees(), angleNeeded);
        double correction = angleDiff*0.16; // rotController.calculate(drivetrain.getGyroRadians(), angleNeeded);
        correction = Util.absClamp(correction, 10);

        drivetrain.setRotationSpeed(correction, 1);
        rotateDone = Math.abs(angleDiff)<3;
        
    }

    double tilt = 76;
    double topRPM = 0;
    double bottomRPM = 0;
    boolean rotateDone = false;
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
        if(rotateDone){
            System.out.println("waiting");

        }
        boolean speedGood = PathUtil.linearSpeedFromChassisSpeeds(drivetrain.getSpeeds())<1;
        if (barrel.readyToShoot() && rotateDone && speedGood && (RobotContainer.leftJoystick.getTrigger() || !manual)) {
            barrel.shoot();
            System.out.println("shot");
            endingTime = System.currentTimeMillis() + 200;
        }

    }

    @Override
    public boolean isFinished() {
        if(manual){
            return false;
        }
        if (System.currentTimeMillis() > endingTime) {
            // cleanup here todo
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
