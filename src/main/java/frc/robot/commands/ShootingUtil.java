package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util;
import frc.robot.commands.paths.PathUtil;
import frc.robot.subsystems.Barrel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class ShootingUtil {
    private static Drivetrain drivetrain;
    private static Barrel barrel;
    private static Vision vision;

    private static double getOffsetPara(double distance, double magnitude){
        double x = 0;
        double result = 0*distance*magnitude;//CURVE:para
        return result;
    }
    private static double getOffsetPerp(double distance, double magnitude){
        double x = 0;
        double result = distance*magnitude;//CURVE:perp
        return result;
    }
    /**
     * Get the current measured distance from target to CAMERA!
     * @return
     */
    public static double getCurrentDistance(){
        double realDist = Util.distance(drivetrain.getPose(), vision.targetToField)-0.35;
        return realDist;// - 0.3048;
    }
    public static double getCurrentLinearSpeed(){
        ChassisSpeeds currentSpeeds = drivetrain.getSpeeds();
        return PathUtil.linearSpeedFromChassisSpeeds(currentSpeeds);
    }
    public static Pose2d getFuturePose(){
        Pose2d currentPose = drivetrain.getPose();
        Pose2d visionPose = vision.targetToField;
        Pose2d futurePose = currentPose;

        ChassisSpeeds currentSpeeds = Util.rotateSpeeds(drivetrain.getSpeeds(), -drivetrain.getGyroRadians());
        double distance = getCurrentDistance();

        EVector2d speedsVector = new EVector2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        EVector2d normalVector = new EVector2d(visionPose.getX() - currentPose.getX(),
                visionPose.getY() - currentPose.getY());
        normalVector.normalize();
        EVector2d perpPart = normalVector.times(speedsVector.dot(normalVector));
        EVector2d paraPart = speedsVector.minus(perpPart);
        //System.out.println("normal"+normalVector);
        //System.out.println("speeds"+speedsVector);
        //System.out.println("dot"+speedsVector.dot(normalVector));

        //System.out.println("perp"+perpPart);
        //System.out.println("para"+paraPart);


        perpPart.setMagnitude(getOffsetPerp(distance, perpPart.magnitude()));
        paraPart.setMagnitude(getOffsetPara(distance, paraPart.magnitude()));
        //System.out.println("perp2"+perpPart);

        var transform = new Transform2d(new Translation2d(perpPart.x+paraPart.x, perpPart.y+paraPart.y), new Rotation2d());
        transform = new Transform2d(new Translation2d(), new Rotation2d());
        //futurePose = futurePose.plus(transform);

        //perpPart.normalized().times();

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
        double bottomRPMDiff = Math.abs(getShootingBottomSpeed(meters)-barrel.getBottomRPM());
        double tiltDiff = Math.abs(getShootingTilt(meters)-barrel.getTiltAngle());
        double turnDiff = Math.abs(Util.angleDiff(drivetrain.getGyroDegrees(), getNeededTurnAngle()));

        double topTime = topRPMDiff * 0.002;
        double bottomTime = bottomRPMDiff*0.002;
        double tiltTime = tiltDiff*0.5;
        double turnTime = turnDiff*(2/180);
        if(meters<=1.5){
            turnTime = 99;
        }else{

        }
        SmartDashboard.putNumber("ShootingRange", meters);

        //System.out.println(topTime);
        //System.out.println(bottomTime);
        //System.out.println(tiltTime);
        //System.out.println(turnTime);
        //System.out.println("next");
        return Math.max(Math.max(Math.max(topTime, bottomTime), tiltTime), turnTime);
    }
    public static void setSubsystems(Drivetrain the_drivetrain, Barrel the_barrel, Vision the_vision) {
        drivetrain = the_drivetrain;
        vision = the_vision;
        barrel = the_barrel;
    }


    public static double getShootingTopSpeed(double meters){
        double x = meters;

        double result = 1.593947969587455*x*x*x + -22.005436231609348*x*x + 161.48336004116533*x + 432.8153682962188; //CURVE:TSPEED,08:49,05/02
        return result;
    }
    public static double getShootingBottomSpeed(double meters){
        double x = meters;

        double result = 3.18789593917491*x*x*x + -44.010872463218696*x*x + 322.96672008233065*x + 865.6307365924376; //CURVE:BSPEED,08:49,05/02
        return result;
    }
    /**
     *
     * @param meters from target's center
     * @return
     */
    public static double getShootingTilt(double meters){
        double x = meters;
        double result = 0.11393176029556593*x*x + -4.871290125522591*x + 82.18778847953466; //CURVE:TILT,10:42,04/23
        if(x<=3.0){
            result = 71.5;
        }
        return result;
    }


}