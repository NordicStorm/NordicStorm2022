package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TurnAndShoot extends CommandBase{

    private Drivetrain drivetrain;
    private Vision vision;
    private long timeout;
    private long endingTime;

    boolean done = false;
    /**
     * Take control of the drivetrain and barrel to take the shot
     * @param drivetrain
     * @param vision
     * @param timeout in milliseconds
     */
    public TurnAndShoot (Drivetrain drivetrain, Vision vision, long timeout){
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.timeout = timeout;
    }

    @Override
    public void initialize() {
        this.endingTime = System.currentTimeMillis() + timeout;
    }

    public void rotateTowardTarget() {
        Pose2d currentPose = drivetrain.getPose();
        Pose2d visionPose = vision.targetToField;
        ChassisSpeeds currentSpeeds = drivetrain.getSpeeds();
        EVector2d speedsVector = new EVector2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        EVector2d normalVector = new EVector2d(visionPose.getX()-currentPose.getX(), visionPose.getY()-currentPose.getY());
        normalVector.normalize();
        EVector2d perpPart = normalVector.multipliedBy(speedsVector.dot(normalVector));
        EVector2d paraPart = speedsVector.minus(perpPart);
        
        
        
        double angleNeeded = Math.toDegrees(Util.angleBetweenPoses(currentPose, vision.targetToField));
        
        double diff = Util.angleDiff(drivetrain.getGyroDegrees(), 0);
        double correction = diff * 0.3;
        correction = Util.absClamp(correction, 2.5);
        drivetrain.setRotationSpeed(correction, 1);
    }
    @Override
    public void execute() {

            rotateTowardTarget();
        
    }


    @Override
    public boolean isFinished() {
        if(System.currentTimeMillis()>endingTime){
            //cleanup here todo
            return true;
        }
        return done;
    }
    
    @Override
    public void end(boolean interrupted) {

    }
}
