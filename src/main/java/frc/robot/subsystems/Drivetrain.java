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

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk3ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.commands.FollowBall;
import frc.robot.commands.paths.DrivetrainConfig;
import frc.robot.commands.paths.PathableDrivetrain;

/**
 *
 */
public class Drivetrain extends SubsystemBase implements PathableDrivetrain {

    private Barrel barrel;
    private Vision vision;
    private Climbers climbers;

    private static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.00;
    //Ticks: -18000
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;
    private List<SwerveModule> swerveModules = new ArrayList<>();
    private DrivetrainConfig drivetrainConfig = new DrivetrainConfig();
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

    private final SwerveDriveOdometry odometry;
    private ChassisSpeeds targetChassisSpeeds;
    private SwerveModuleState currentSwerveStates [] = new SwerveModuleState[4];
    private Pose2d pose;
    private final AHRS navx = new AHRS(Port.kMXP);

    // this is basically the 'privilege level' the rotation control ability is at.
    // So 0 means it will take anything, 1 means 1 or higher.
    // when a raw "drive" is used, privilege level is 0.
    // Each tick, whatever thing gave the highest rotation privilege gets used.
    private int currentRotationPrivilegeNeeded = 0;
    
    Pixy pixy;
    public int myBallColor = 0;
    public int enemyBallColor = 0;

    public Field2d fieldDisplay;
    public Drivetrain() {
        pixy = new Pixy();
        pixy.startUpdatingPixy();
        setBallColors();
        fieldDisplay = new Field2d();
        SmartDashboard.putData(fieldDisplay);

        frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                null, new Mk3ModuleConfiguration(),
                Mk3SwerveModuleHelper.GearRatio.FAST, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                Constants.FRONT_LEFT_MODULE_STEER_MOTOR, Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
                Constants.FRONT_LEFT_MODULE_STEER_OFFSET);

        frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
                null, new Mk3ModuleConfiguration(),
                Mk3SwerveModuleHelper.GearRatio.FAST, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.FRONT_RIGHT_MODULE_STEER_MOTOR, Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);

        backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                null, new Mk3ModuleConfiguration(),
                Mk3SwerveModuleHelper.GearRatio.FAST, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                Constants.BACK_LEFT_MODULE_STEER_MOTOR, Constants.BACK_LEFT_MODULE_STEER_ENCODER,
                Constants.BACK_LEFT_MODULE_STEER_OFFSET);

        backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                null, new Mk3ModuleConfiguration(),
                Mk3SwerveModuleHelper.GearRatio.FAST, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.BACK_RIGHT_MODULE_STEER_MOTOR, Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
                Constants.BACK_RIGHT_MODULE_STEER_OFFSET);
        swerveModules.add(frontLeftModule);
        swerveModules.add(frontRightModule);
        swerveModules.add(backLeftModule);
        swerveModules.add(backRightModule);

        for (SwerveModule module : swerveModules) {
            TalonFX driveMotor = module.getTalonDriveMotor();

            driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
            driveMotor.config_kF(0, 0.048);
            driveMotor.config_kP(0, 0.04);
            
        }
        drivetrainConfig.maxAcceleration = 3; 
        drivetrainConfig.maxVelocity = 4; 
        drivetrainConfig.maxAnglularVelocity = 10;
        drivetrainConfig.maxAngularAcceleration = 5;
        drivetrainConfig.rotationCorrectionP = 2;
        drivetrainConfig.maxCentripetalAcceleration = 8;

        pose = new Pose2d(6, 4, Rotation2d.fromDegrees(0));
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(0), pose);
        //SmartDashboard.putNumber("MaxAccel", 4);
        targetChassisSpeeds = new ChassisSpeeds(0, 0, 0);
        drive(0, 0, 0);

    }
    public void setOtherSubsystems(Vision vision, Climbers climbers, Barrel barrel){
        this.vision = vision;
        this.barrel = barrel;
        this.climbers = climbers;
    }
    public void setBallColors(){
        if(DriverStation.getAlliance() == Alliance.Blue){
            myBallColor = 1;
            enemyBallColor = 2;
        }else{
            myBallColor= 2;
            enemyBallColor = 1;
        }
    }
    public void zeroGyroscope() {
        navx.zeroYaw();
    }

    /**
     * Goes positive as it goes counterclockwise. Degrees!
     * 
     * @return current angle in degrees
     */
    public double getGyroDegrees() {
        return -navx.getAngle();
    }

    @Override
    public double getGyroRadians() {
        return Math.toRadians(getGyroDegrees());
    }

    @Override
    public Pose2d getPose() {

        return pose;
    }
    /**
     * Resets the position
     * @param x
     * @param y
     * @param rot in radians, but this is ignored.
     */
    public void setPose(double x, double y, double rot) {
        // navx.reset();
        odometry.resetPosition(new Pose2d(x, y, new Rotation2d(rot)), new Rotation2d(rot));
    }

    public void resetAngle() {
        navx.reset();
        navx.setAngleAdjustment(0);
        
    }
    /**
     * This will get added to the angle result to offset it.
     * A positive
     * value means that the robot is pointing x degrees counterclockwise
     * @param degrees
     */
    public void setAngleOffset(double degrees){
        navx.setAngleAdjustment(-degrees);
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(currentSwerveStates);
    }

    @Override
    public DrivetrainConfig getConfig() {
        return drivetrainConfig;
    }

    public Pixy getPixy() {
        return pixy;
    }

    @Override
    public void drive(ChassisSpeeds chassisSpeeds) {
        drive(chassisSpeeds, 0);
    }

    @Override
    public void periodic() {

        for(int i = 0; i<swerveModules.size(); i++){
            currentSwerveStates[i]=Util.stateFromModule(swerveModules.get(i));
        }
        // Update the pose
        pose = odometry.update(Rotation2d.fromDegrees(getGyroDegrees()), currentSwerveStates);
        
        driveActualMotors(targetChassisSpeeds);
        currentRotationPrivilegeNeeded = 0;
        fieldDisplay.setRobotPose(pose.getX(), pose.getY(), new Rotation2d(getGyroRadians()));
        SmartDashboard.putNumber("Pitch", navx.getRoll());

        SmartDashboard.putNumber("driveAng", getGyroDegrees());
        SmartDashboard.putNumber("Pixy Num", FollowBall.countTargets( pixy.readObjects(), myBallColor));
        if (RobotContainer.rightJoystick.getRawButton(12)) {
            resetAngle();
        }
        
    }

    public void drive(ChassisSpeeds chassisSpeeds, int rotPrivilege) {
        targetChassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond;
        targetChassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond;
        if (rotPrivilege >= currentRotationPrivilegeNeeded) {
            currentRotationPrivilegeNeeded = rotPrivilege;
            targetChassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond;
        }
    }

    private void driveActualMotors(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        frontLeftModule.setWithVelocity(states[0].speedMetersPerSecond,
                states[0].angle.getRadians());
        frontRightModule.setWithVelocity(states[1].speedMetersPerSecond,
                states[1].angle.getRadians());
        backLeftModule.setWithVelocity(states[2].speedMetersPerSecond,
                states[2].angle.getRadians());
        backRightModule.setWithVelocity(states[3].speedMetersPerSecond,
                states[3].angle.getRadians());
    }

    public void limitDrive(ChassisSpeeds localSpeeds, int rotPrivilege) {
        boolean vWalls = false;// Robot.vision.hasSeenTarget;
        var currentLocalSpeeds = getSpeeds();

        double maxAccelLocal = 3;
        localSpeeds.vxMetersPerSecond = doAccelerationLimit(currentLocalSpeeds.vxMetersPerSecond,
                localSpeeds.vxMetersPerSecond, maxAccelLocal, maxAccelLocal);
        localSpeeds.vyMetersPerSecond = doAccelerationLimit(currentLocalSpeeds.vyMetersPerSecond,
                localSpeeds.vyMetersPerSecond, maxAccelLocal, maxAccelLocal);

        var targetFieldSpeeds = Util.rotateSpeeds(localSpeeds, -getGyroRadians());

        double fixX = enforceWalls(targetFieldSpeeds.vxMetersPerSecond, drivetrainConfig.maxAcceleration,
                pose.getX(), 1, 4.2);
        if (vWalls){
            targetFieldSpeeds.vxMetersPerSecond = fixX;
        }
        double fixY = enforceWalls(targetFieldSpeeds.vyMetersPerSecond, drivetrainConfig.maxAcceleration,
                pose.getY(), -6.5, -1);
        if (vWalls){
            targetFieldSpeeds.vyMetersPerSecond = fixY;
        }
        var targetLocalSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetFieldSpeeds.vxMetersPerSecond,
                targetFieldSpeeds.vyMetersPerSecond,
                targetFieldSpeeds.omegaRadiansPerSecond,
                Rotation2d.fromDegrees(getGyroDegrees()));

        drive(targetLocalSpeeds, rotPrivilege);

        // Robot.drivetrain.drive(forward * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
        // sideways * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, rot *
        // Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

    }

    /**
     * 
     * @param current
     * @param target
     * @param plusLimit  meters per second accel
     * @param minusLimit
     * @return
     */
    double doAccelerationLimit(double current, double target, double plusLimit, double minusLimit) {
        minusLimit *= -1;
        double factor = 0.25; // trial and error to find this
        plusLimit *= factor;
        minusLimit *= factor;
        var accelNeeded = (target - current);
        // in 0.02 second, it is trying to change by ^
        // System.out.println("accelNeed: " + accelNeeded);
        if (accelNeeded > plusLimit) {
            return current + plusLimit;
        }
        if (accelNeeded < minusLimit) {
            return current + minusLimit;
        }
        return target;
    }

    double enforceWalls(double targetSpeed, double maxAccel, double currentPos, double min, double max) {
        if (targetSpeed > 0) {
            targetSpeed = enforceSingleWall(targetSpeed, maxAccel, currentPos, max);
        } else {
            targetSpeed = -enforceSingleWall(-targetSpeed, maxAccel, -currentPos, -min);
        }

        return targetSpeed;
    }

    double enforceSingleWall(double targetSpeed, double maxAccel, double currentPos, double max) {
        double dist = max - currentPos;

        targetSpeed = Math.min(maxAccel * dist * 1.5, targetSpeed);

        return targetSpeed;
    }

    /**
     * Raw drive the motors, units in VOLTS!
     * @param chassisSpeeds
     */
    public void driveVolts(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        frontLeftModule.set(states[0].speedMetersPerSecond,
                states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond,
                states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond,
                states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond,
                states[3].angle.getRadians());
    }

    public void drive(double x, double y, double rot) {
        drive(new ChassisSpeeds(x, y, rot));
    }

    /**
     * 
     * @param speed        radians per second CCW
     * @param rotPrivilege
     */
    public void setRotationSpeed(double speed, int rotPrivilege) {
        if (rotPrivilege >= currentRotationPrivilegeNeeded) {
            targetChassisSpeeds.omegaRadiansPerSecond = speed;
            currentRotationPrivilegeNeeded = rotPrivilege;
        }
    }

    /**
     * 
     * @return meters away from the center of the target
     */
    public double getDistanceToTarget(){
        return Util.distance(pose, vision.targetToField);
    }
    @Override
    public void setPose(Pose2d pose) {
        setPose(pose.getX(), pose.getY(), pose.getRotation().getRadians());
        
    }

    public void resetSwerve(){
        for(int i = 0; i<600; ++i){
            driveActualMotors(new ChassisSpeeds());
        }
    }


}
