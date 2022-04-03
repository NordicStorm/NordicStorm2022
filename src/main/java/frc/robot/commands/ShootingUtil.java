package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util;
import frc.robot.subsystems.Barrel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class ShootingUtil {
    private static Drivetrain drivetrain;
    private static Barrel barrel;
    private static Vision vision;

    private static double getOffsetPara(double distance, double magnitude){
        double x = 0;
        double result = 0;//CURVE:para
        return result;
    }
    private static double getOffsetPerp(double distance, double magnitude){
        double x = 0;
        double result = 0;//CURVE:perp
        return result;
    }
    /**
     * Get the current measured distance from target to CAMERA!
     * @return
     */
    public static double getCurrentDistance(){
        return Util.distance(drivetrain.getPose(), vision.targetToField)-0.35;
    }
    public static Pose2d getFuturePose(){
        Pose2d currentPose = drivetrain.getPose();
        Pose2d visionPose = vision.targetToField;
        ChassisSpeeds currentSpeeds = drivetrain.getSpeeds();
        double distance = getCurrentDistance();

        EVector2d speedsVector = new EVector2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        EVector2d normalVector = new EVector2d(visionPose.getX() - currentPose.getX(),
                visionPose.getY() - currentPose.getY());
        normalVector.normalize();
        EVector2d perpPart = normalVector.times(speedsVector.dot(normalVector));
        EVector2d paraPart = speedsVector.minus(perpPart);

        //perpPart.normalized().times();

        Pose2d futurePose = currentPose;
        return futurePose;
    }
    /**Returns the angle needed in degrees */
    public static double getNeededTurnAngle(){
        Pose2d futurePose = ShootingUtil.getFuturePose();

        double angleNeeded = Util.angleBetweenPoses(futurePose, vision.targetToField)+Math.PI;
        return Math.toDegrees(angleNeeded);

    }
    public static double getTimeToReady(){
        double meters = getCurrentDistance();
        double topRPMDiff = Math.abs(getShootingTopSpeed(meters)-barrel.getTopRPM());
        double bottomRPMDiff = Math.abs(getShootingBottomSpeed(meters));
        double tiltDiff = Math.abs(getShootingTilt(meters));
        double turnDiff = Math.abs(Util.angleDiff(drivetrain.getGyroDegrees(), getNeededTurnAngle()));

        double topTime = topRPMDiff * 0.002;
        double bottomTime = bottomRPMDiff*0.002;
        double tiltTime = tiltDiff*0.5;
        double turnTime = turnDiff*(1/180);
        return Math.max(Math.max(Math.max(topTime, bottomTime), tiltTime), turnTime);
    }
    public static void setSubsystems(Drivetrain the_drivetrain, Barrel the_barrel, Vision the_vision) {
        drivetrain = the_drivetrain;
        vision = the_vision;
        barrel = the_barrel;
    }


    public static double getShootingTopSpeed(double meters){
        double x = meters;

        double result = -23.949002766088412*x*x + 342.51851904890225*x + -58.80671457761888; //CURVE:TSPEED,12:01,04/03
        return result;
    }
    public static double getShootingBottomSpeed(double meters){
        double x = meters;

        double result = 3.2330871259301976*x*x*x*x + -59.81547106179853*x*x*x + 357.15000710874875*x*x + -606.0028359089331*x + 1792.9892330663902; //CURVE:BSPEED,12:01,04/03
        return result;
    }
    /**
     *
     * @param meters from target's center
     * @return
     */
    public static double getShootingTilt(double meters){
        double x = meters;
        double result = -0.0937014192731562*x*x*x*x*x + 2.2437772636000606*x*x*x*x + -20.59687229539596*x*x*x + 89.34733054408112*x*x + -183.61201620232927*x + 215.42450215660352; //CURVE:TILT,12:01,04/03
        return result;
    }
}