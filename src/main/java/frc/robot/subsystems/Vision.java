// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util;
import frc.robot.commands.RollingAverage;

/**
 *
 */
public class Vision extends SubsystemBase {

    private Drivetrain drivetrain;
    private Barrel barrel;
    private Climbers climbers;

    PhotonCamera camera;

    public Vision() {
        camera = new PhotonCamera("picam");
        setLight(true);

    }

    private double lastDistance = 0;
    private RollingAverage distanceAverage = new RollingAverage(5);
    public boolean canSeeTarget = false;
    double targetCenter = 0;
    Pose2d pose = null;
    boolean lightOn = true;
    double camHeight = Units.inchesToMeters(27); // 27
    double targetHeight = Units.inchesToMeters(104); // 104
    double camAngle = Math.toRadians(29.7);

    // Position of robot relative to cam
    Transform2d camToRobot = new Transform2d(new Translation2d(Units.inchesToMeters(-13.75), 0),
            Rotation2d.fromDegrees(0));
    final public Pose2d targetToField = new Pose2d(Units.feetToMeters(27), Units.feetToMeters(13.5), new Rotation2d(0));
    public PhotonTrackedTarget bestTarget = null;
    public boolean hasSeenTarget = false;

    @Override
    public void periodic() {
        var res = camera.getLatestResult();
        // System.out.println(res);
        if (res.hasTargets()) {
            List<PhotonTrackedTarget> targets = res.getTargets();

            var firstBest = targets.get(0);
            if (targets.size() > 1) {

                var secondBest = targets.get(1);
                if (Math.abs(firstBest.getPitch() - secondBest.getPitch()) < 0.5) {
                    if (firstBest.getYaw() < secondBest.getYaw()) {
                        bestTarget = firstBest;
                    } else {
                        bestTarget = secondBest;
                    }
                } else {
                    bestTarget = firstBest;
                }
            }else{
                bestTarget = firstBest;
            }
            double visYaw = bestTarget.getYaw();
            double visPitch = bestTarget.getPitch();
            if(visPitch<20){
                SmartDashboard.putNumber("visYaw", visYaw);
            }
            boolean usable = Math.abs(visYaw) < 4.5 && visPitch < 20;
            canSeeTarget = true;

            if (usable) {
                SmartDashboard.putNumber("visUpdate", Math.random());
                double recentDistance = PhotonUtils.calculateDistanceToTargetMeters(
                        camHeight,
                        targetHeight,
                        camAngle,
                        Math.toRadians(bestTarget.getPitch())) + 0.68;
                Rotation2d botRotation = Rotation2d.fromDegrees(drivetrain.getGyroDegrees() + 180);
                recentDistance = visToRealDist(recentDistance);
                distanceAverage.put(recentDistance);
                lastDistance = distanceAverage.get();

                var estPose = estimateFieldToRobot(lastDistance,
                        Rotation2d.fromDegrees(-bestTarget.getYaw()), botRotation, targetToField,
                        camToRobot);

                // SmartDashboard.putNumber("vis_dist_in", Units.metersToInches(lastDistance));
                // SmartDashboard.putNumber("vis_dist", lastDistance);

                // SmartDashboard.putNumber("vis_x", estPose.getX());
                // SmartDashboard.putNumber("vis_y", estPose.getY());

                // if(Util.distance(estPose, drivetrain.getPose())<3 || !hasSeenTarget){
                drivetrain.setPose(estPose.getX(), estPose.getY(), 0);
                // }else{
                // SmartDashboard.putString("Message", "Vision desync");
                // }
                // System.out.println("dist "+);
                hasSeenTarget = true;

            } else {
                distanceAverage.clear();
            }
        } else {
            distanceAverage.clear();
            canSeeTarget = false;
            SmartDashboard.putNumber("visYaw", 0);

            // System.out.println("no target");
        }
    }

    public static double visToRealDist(double distanceV) {
        double x = distanceV;
        double result = 1.1608113266065743 * x + -0.21172960362220683; // CURVE:distance,08:13,03/29
        return result;
    }

    public void setOtherSubsystems(Drivetrain drivetrain, Climbers climbers, Barrel barrel) {
        this.drivetrain = drivetrain;
        this.barrel = barrel;
        this.climbers = climbers;
    }

    public void toggleLight() {
        setLight(!lightOn);
    }

    public void setLight(boolean on) {
        lightOn = on;
        camera.setLED(on ? VisionLEDMode.kDefault : VisionLEDMode.kOff);
    }

    public Pose2d getEstimatedPose() {
        return pose;
    }

    /**
     * Estimate the position of the robot in the field. Adds to the distance so it
     * represents the center of the target
     *
     * @param distanceToCenter The distance from the camera to the center of the
     *                         target
     * 
     * @param targetYaw        The observed yaw of the target. Note that this
     *                         *must* be CCW-positive, and
     *                         Photon returns CW-positive.
     * @param gyroAngle        The current robot gyro angle, likely from odometry.
     * @param fieldToTarget    A Pose2d representing the target position in the
     *                         field coordinate system.
     * @param cameraToRobot    The position of the robot relative to the camera.
     *                         If the camera was
     *                         mounted 3 inches behind the "origin" (usually
     *                         physical center) of the robot, this would be
     *                         Transform2d(3 inches, 0 inches, 0 degrees).
     * @return The position of the robot in the field.
     */
    public static Pose2d estimateFieldToRobot(
            double distanceToCenter,
            Rotation2d targetYaw,
            Rotation2d gyroAngle,
            Pose2d fieldToTarget,
            Transform2d cameraToRobot) {
        return PhotonUtils.estimateFieldToRobot(
                PhotonUtils.estimateCameraToTarget(
                        PhotonUtils.estimateCameraToTargetTranslation(
                                distanceToCenter,
                                targetYaw),
                        fieldToTarget,
                        gyroAngle),
                fieldToTarget,
                cameraToRobot);
    }

    public void resetCam() {
        setLight(true);
        camera.setPipelineIndex(2);
    }
}