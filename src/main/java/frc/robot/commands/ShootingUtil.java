package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    public static Pose2d getFuturePose(){
        Pose2d currentPose = drivetrain.getPose();
        Pose2d visionPose = vision.targetToField;
        ChassisSpeeds currentSpeeds = drivetrain.getSpeeds();
        double distance = vision.lastDistance;

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
    public static double getNeededTurnAngle(){
        Pose2d futurePose = ShootingUtil.getFuturePose();

        double angleNeeded = Math.toDegrees(Util.angleBetweenPoses(futurePose, vision.targetToField));
        return angleNeeded;

    }
    public static double getTimeToReady(){
        double meters = vision.lastDistance; //TODO
        double topRPMDiff = Math.abs(getShootingTopSpeed(meters)-barrel.getTopRPM());
        double bottomRPMDiff = Math.abs(getShootingBottomSpeed(meters));
        double tiltDiff = Math.abs(getShootingTilt(meters));

        double turnDiff = Math.abs(Util.angleDiff(drivetrain.getGyroDegrees(), getNeededTurnAngle()));


        return 999;
    }
    public static void setSubsystems(Drivetrain the_drivetrain, Barrel the_barrel, Vision the_vision) {
        drivetrain = the_drivetrain;
        vision = the_vision;
        barrel = the_barrel;
    }


    public static double getShootingTopSpeed(double meters){
        double x = meters;

        double result = -20.748448193695662*x*x + 311.9649788180637*x + 0.3237476579643795; //CURVE:TSPEED,11:29,04/02
        return result;
    }
    public static double getShootingBottomSpeed(double meters){
        double x = meters;

        double result = -32.060765399047035*x*x + 437.4155079954175*x + 812.5758207596833; //CURVE:BSPEED,11:29,04/02
        return result;
    }
    /**
     *
     * @param meters from target's center
     * @return
     */
    public static double getShootingTilt(double meters){
        double x = meters;
        double result = -0.09229764095572934*x*x*x*x*x + 2.20533797489179*x*x*x*x + -20.193627209804923*x*x*x + 87.335434603262*x*x + -178.8701042643679*x + 211.20395403399414; //CURVE:TILT,11:29,04/02
        return result;
    }
}