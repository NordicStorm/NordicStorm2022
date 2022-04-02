package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.subsystems.Barrel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TurnAndShoot extends CommandBase {

    private Drivetrain drivetrain;
    private Barrel barrel;
    private Vision vision;
    private long timeout;
    private long endingTime;

    boolean done = false;
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
        SmartDashboard.putNumber("sTilt", 79.6);
        SmartDashboard.putNumber("sTop", 1000);
        SmartDashboard.putNumber("sBottom", 1000);
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
        if (SmartDashboard.getBoolean("go", false)) {
            SmartDashboard.putBoolean("go", false);
            tilt = SmartDashboard.getNumber("sTilt", 76.9);
            topRPM = SmartDashboard.getNumber("sTop", 1000);
            bottomRPM = SmartDashboard.getNumber("sBottom", 1000);
            SmartDashboard.putString("csv", vision.lastDistance+","+bottomRPM+","+topRPM+","+175); //175 fake angles
            barrel.setFlywheels(topRPM, bottomRPM);
        }
        // barrel.setTiltAngle(titl);
        //rotateTowardTarget();
        // barrel.setTiltFromVision();

        if (barrel.readyToShoot() && RobotContainer.leftJoystick.getTrigger()) {
            barrel.shoot();
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