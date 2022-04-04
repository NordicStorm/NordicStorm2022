package frc.robot.commands.paths;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PivotPiece extends CommandBase implements CommandPathPiece {

    private PathableDrivetrain drivetrain;
    private DrivetrainConfig drivetrainConfig;
    private HolonomicDriveController controller;

    PivotPiece(PathableDrivetrain drivetrain, List<WaypointPiece> waypoints, double endVelocity,
            MultiPartPath path) {
        this.drivetrain = drivetrain;
 
        this.drivetrainConfig = path.getDrivetrainConfig();
    }


    @Override
    public double getRequestedStartSpeed() {
        return 0; // TODO?
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drivetrain.getPose();
        ChassisSpeeds currentSpeeds = drivetrain.getSpeeds();
        double speed = PathUtil.linearSpeedFromChassisSpeeds(currentSpeeds);
        ChassisSpeeds globalSpeeds = PathUtil.rotateSpeeds(currentSpeeds, drivetrain.getGyroRadians());
        List<Translation2d> interiorPoints = new ArrayList<>();
        Rotation2d startMovementDirection = null;


    }

    @Override
    public void execute() {
        

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
