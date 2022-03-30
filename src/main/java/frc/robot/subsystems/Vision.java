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
        camera.setPipelineIndex(2);
    }

    double distance = 0;
    double targetCenter = 0;
    Pose2d pose = null;
    boolean lightOn = true;
    double camHeight = Units.inchesToMeters(27); //27
    double targetHeight = Units.inchesToMeters(104); //104
    double camAngle = Math.toRadians(29.7);

    // Position of robot relative to cam
    Transform2d camToRobot = new Transform2d(new Translation2d(Units.inchesToMeters(12), 0),
            Rotation2d.fromDegrees(0));
    final public Pose2d targetToField = new Pose2d(Units.feetToMeters(27), Units.feetToMeters(13.5), new Rotation2d(0));
    public PhotonTrackedTarget bestTarget = null;
    public boolean hasSeenTarget = false;

    @Override
    public void periodic() {
        var res = camera.getLatestResult();
        // System.out.println(res);
        if (res.hasTargets()) {
            PhotonTrackedTarget target = res.getBestTarget();
            bestTarget = target;
            double visYaw = target.getYaw();
            double visPitch = target.getPitch();
            boolean usable = Math.abs(visYaw) < 4.5;
            if (usable) {
                hasSeenTarget = true;
                SmartDashboard.putNumber("visUpdate", Math.random());
                double distance = PhotonUtils.calculateDistanceToTargetMeters(
                        camHeight,
                        targetHeight,
                        camAngle,
                        Math.toRadians(res.getBestTarget().getPitch()))+0.68;
                Rotation2d botRotation = Rotation2d.fromDegrees(drivetrain.getGyroDegrees()+180);
                var estPose = estimateFieldToRobot(
                        camHeight, targetHeight, camAngle, Math.toRadians(target.getPitch()),
                        Rotation2d.fromDegrees(-target.getYaw()), botRotation, targetToField,
                        camToRobot);

                SmartDashboard.putNumber("vis_dist", Units.metersToInches(distance));

                SmartDashboard.putNumber("vis_x", estPose.getX());
                SmartDashboard.putNumber("vis_y", estPose.getY());
                SmartDashboard.putNumber("vis_x_ft", Units.metersToFeet(estPose.getX()));
                SmartDashboard.putNumber("vis_y_ft", Units.metersToFeet(estPose.getY()));
                drivetrain.resetPose(estPose.getX(), estPose.getY(), 0);
                // System.out.println("dist "+);
            }
        } else {
            // System.out.println("no target");
        }
    }

    public double visToRealDist(double distanceV){
        double x = distanceV;
        double result = 0;//CURVE:distance
        return result;
    }
    public void setOtherSubsystems(Drivetrain drivetrain, Climbers climbers, Barrel barrel){
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

    public double getCenterOfTarget() {
        return 0;
    }

    public Pose2d getEstimatedPose() {
        return pose;
    }

    /**
     * Estimate the position of the robot in the field. Adds to the distance so it
     * represents the center of the target
     *
     * @param cameraHeightMeters The physical height of the camera off the floor in
     *                           meters.
     * @param targetHeightMeters The physical height of the target off the floor in
     *                           meters. This
     *                           should be the height of whatever is being targeted
     *                           (i.e. if the targeting region is set to
     *                           top, this should be the height of the top of the
     *                           target).
     * @param cameraPitchRadians The pitch of the camera from the horizontal plane
     *                           in radians.
     *                           Positive values up.
     * @param targetPitchRadians The pitch of the target in the camera's lens in
     *                           radians. Positive
     *                           values up.
     * @param targetYaw          The observed yaw of the target. Note that this
     *                           *must* be CCW-positive, and
     *                           Photon returns CW-positive.
     * @param gyroAngle          The current robot gyro angle, likely from odometry.
     * @param fieldToTarget      A Pose2d representing the target position in the
     *                           field coordinate system.
     * @param cameraToRobot      The position of the robot relative to the camera.
     *                           If the camera was
     *                           mounted 3 inches behind the "origin" (usually
     *                           physical center) of the robot, this would be
     *                           Transform2d(3 inches, 0 inches, 0 degrees).
     * @return The position of the robot in the field.
     */
    public static Pose2d estimateFieldToRobot(
            double cameraHeightMeters,
            double targetHeightMeters,
            double cameraPitchRadians,
            double targetPitchRadians,
            Rotation2d targetYaw,
            Rotation2d gyroAngle,
            Pose2d fieldToTarget,
            Transform2d cameraToRobot) {
        return PhotonUtils.estimateFieldToRobot(
                PhotonUtils.estimateCameraToTarget(
                        PhotonUtils.estimateCameraToTargetTranslation(
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        cameraHeightMeters, targetHeightMeters,
                                        cameraPitchRadians, targetPitchRadians)
                                        + 0.68,
                                targetYaw),
                        fieldToTarget,
                        gyroAngle),
                fieldToTarget,
                cameraToRobot);
    }
}
