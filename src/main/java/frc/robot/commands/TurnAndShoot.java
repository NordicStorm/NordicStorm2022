package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.commands.paths.CommandPathPiece;
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
    Constraints constraints = new Constraints(2.5, 2);
    ProfiledPIDController rotController = new ProfiledPIDController(0.05, 0, 0, constraints);

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

        rotController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

    }

    @Override
    public void initialize() {
        this.endingTime = System.currentTimeMillis() + timeout;
    }
    
    public void rotateTowardTarget() {
        double angleNeeded = ShootingUtil.getNeededTurnAngle();


        double correction = -rotController.calculate(drivetrain.getGyroRadians(), angleNeeded);
        correction = Util.absClamp(correction, 2.5);
        drivetrain.setRotationSpeed(correction, 1);
        
    }

    double tilt = 76;
    double topRPM = 0;
    double bottomRPM = 0;

    @Override
    public void execute() {
        double lastDistance = vision.lastDistance;
        if (SmartDashboard.getBoolean("go", false) && manual) {
            SmartDashboard.putBoolean("go", false);
            tilt = SmartDashboard.getNumber("sTilt", barrel.intakePos);
            bottomRPM = SmartDashboard.getNumber("sBottom", 1000);
            topRPM = bottomRPM*SmartDashboard.getNumber("spin", 0.6);;
            
            SmartDashboard.putNumber("sTop", topRPM);
            //distance is all to lense!
            SmartDashboard.putString("csv", vision.lastDistance+","+bottomRPM+","+topRPM+","+tilt); //175 fake angles
            barrel.setFlywheels(topRPM, bottomRPM);
            barrel.setTiltAngle(tilt);
        }else{
            
        }
        if(RobotContainer.leftJoystick.getRawButton(5)){
            topRPM = ShootingUtil.getShootingTopSpeed(lastDistance);
            bottomRPM = ShootingUtil.getShootingBottomSpeed(lastDistance);
            tilt = ShootingUtil.getShootingTilt(lastDistance);
            barrel.setFlywheels(topRPM, bottomRPM);
            barrel.setTiltAngle(tilt);
            SmartDashboard.putNumber("sTilt", tilt);
            SmartDashboard.putNumber("sTop", topRPM);
            SmartDashboard.putNumber("sBottom", bottomRPM);
            SmartDashboard.putNumber("spin", topRPM/bottomRPM);
        }
        

        if(!manual){
            barrel.setFlywheels(500, 500);
        }
        //rotateTowardTarget();
        // barrel.setTiltFromVision();

        if (barrel.readyToShoot() && (RobotContainer.leftJoystick.getTrigger() || !manual)) {
            barrel.shoot();
            done = true;
        }

    }

    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() > endingTime) {
            // cleanup here todo
            return true;
        }
        return done;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
