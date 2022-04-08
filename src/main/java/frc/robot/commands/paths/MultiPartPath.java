package frc.robot.commands.paths;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.paths.PathPiece.PieceType;

/**
 * This class should be used instead of WPILib CommandGroup for the main
 * autonomous. Used for making a smooth, continous motion with some actions
 * along the way. All units are in meters or degrees!
 * <p>
 * Coordinates are all in meters. The axis directions are generally oriented
 * based on what alliance you are on.
 * <p>
 * Everywhere, x represents the axis the long way on the field, eg between the
 * red and blue alliance stations. Positive x means forward! Toward the opposite
 * alliance's station.
 * <p>
 * While y represents the short axis, between the two side walls. Positive y
 * means to the left, from your driver station's perspective.
 * <p>
 * It's like you are looking at the field from the stands.
 * <p>
 * In most cases, angle is in degrees. Positive = counterclockwise.
 */
public class MultiPartPath {

    double headingOffset;
    boolean headingFollowMovement;
    double targetRotationDegrees;

    PathableDrivetrain drivetrain;
    DrivetrainConfig drivetrainConfig;
    ProfiledPIDController rotationController;

    private MultiPartPath parent;
    //contains the piece and if it interrupts
    private List<Pair<PathPiece, Boolean>> pieces = new ArrayList<>();
    /**
     * Constructs a path.
     * 
     * @param drivetrain the drivetrain that the path will use to move.
     * @see MultiPartPath
     */
    public MultiPartPath(PathableDrivetrain drivetrain) {
        this(drivetrain, drivetrain.getConfig().makeClone(), null);

    }

    public MultiPartPath(PathableDrivetrain drivetrain, DrivetrainConfig config, MultiPartPath parent) {
        this.drivetrainConfig = config;

        rotationController = new ProfiledPIDController(drivetrainConfig.rotationCorrectionP,
                drivetrainConfig.rotationCorrectionI, drivetrainConfig.rotationCorrectionD,
                new TrapezoidProfile.Constraints(drivetrainConfig.maxAnglularVelocity,
                        drivetrainConfig.maxAngularAcceleration));
        rotationController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
        this.drivetrain = drivetrain;
        this.parent = parent;
    }

    /**
     * Move to this point, following a smooth curve from the previous position.
     * 
     * @param x meters
     * @param y meters
     */
    public void addWaypoint(double x, double y) {
        pieces.add(new Pair<>(new WaypointPiece(x, y), false));
    }

    /**
     * If the piece will have an effect on the trajectory, so it needs to be
     * interrupted. You should use addParallelCommand if it is an instant instruction, such as setting
     * shooter speed.
     * 
     */

    public void addSequentialCommand(CommandPathPiece command) {
        pieces.add(new Pair<>(command, true));
    }
    
    /**
     * Wraps a plain command so it can be used in the path
     * @param command
     */
    public void addSequentialCommand(Command command, double startSpeed) {
        addSequentialCommand(new PathPieceWrapper(command, startSpeed));
    }
    /**
     * If the piece will not have an effect on the trajectory, so it can keep going.
     * You should use addSequentialCommand if it is an driving instruction, such as 
     * following a ball.
     * 
     */

    public void addParallelCommand(CommandPathPiece command) {
        pieces.add(new Pair<>(command, false));
    }
    /**
     * Bring the robot to a complete stop, and wait for the specified number of
     * milliseconds.
     * 
     * @param milliseconds Milliseconds. 1000ms = 1 second
     */
    public void stop(int milliseconds) {
        addSequentialCommand(new FullStopPiece(this, milliseconds));
    }

    /**
     * Stop completely.
     * 
     * see {@link #stop(int)}
     */
    public void stop() {
        stop(0);
    }

    /**
     * Set the heading that the robot wants to achieve. Note, this will NOT just
     * stop and pivot to there. It will take effect when moving between waypoints.
     * To stop and pivot, use the pivotInPlace method (todo).
     * 
     * @param degrees
     */
    public void setHeading(double degrees) {
        addParallelCommand(new HeadingSetPiece(this, degrees, false));
    }

    /**
     * Make the robot rotate in the direction it is moving. It will only take effect
     * when moving between waypoints.
     * 
     * @param offsetDegrees an offset from the direction the robot is moving.
     *                      Positive is counterclockwise
     */
    public void setHeadingFollowMovement(double offsetDegrees) {
        addParallelCommand(new HeadingSetPiece(this, offsetDegrees, true));
    }

    /**
     * When this part is run, the maxVelocity on the drivetrainConfig will be
     * changed. This will affect the speed while moving in a trajectory between
     * waypoints, but it will not be enforced for user commands.
     * 
     * @param maxVelocity new max speed in meters per second.
     */
    public void changeMaxVelocity(double maxVelocity) {
        changeDrivetrainConfigProperty("maxVelocity", maxVelocity);
    }

    /**
     * When this part is run, the maxAcceleration on the drivetrainConfig will be
     * changed. This will affect the acceleration while moving in a trajectory
     * between waypoints, but it will not be enforced for user commands.
     * 
     * @param maxAcceleration new max acceleration in meters per second per second.
     */
    public void changeMaxAcceleration(double maxAcceleration) {
        changeDrivetrainConfigProperty("maxAcceleration", maxAcceleration);
    }

    /**
     * When this is run, it will change the given value in the drivetrainConfig. For
     * velocity or acceleration, use the changeMaxVelocity/Acceleration methods.
     * This piece allows you to change other properties if needed.
     * 
     * @param name the name of the property in the drivetrainConfig. 
     * @param value the new value of the property
     */
    public void changeDrivetrainConfigProperty(String name, double value) {
        addParallelCommand(new ConfigPropertySetPiece(this, name, value));
    }

    /**
     * When this part is run, it will overwrite where the robot thinks it is
     * with the new coordinates.
     * @param x meters
     * @param y meters
     */
    public void resetPosition(double x, double y){
        addParallelCommand(new ResetPosePiece(this, new Pose2d(x, y, new Rotation2d()))); // the rot 
    }

    public SequentialCommandGroup finalizePath() {
        SequentialCommandGroup group = new SequentialCommandGroup();
        List<WaypointPiece> waypoints = new ArrayList<>();
        List<Command> actualCommands = new ArrayList<>();
        for (var pieceInfo : pieces) {
            var piece = pieceInfo.getFirst();
            boolean interrupts = pieceInfo.getSecond();
            if (piece.getPieceType() == PieceType.Waypoint) {
                waypoints.add((WaypointPiece) piece);
            } else {
                var commandPiece = (CommandPathPiece) piece;
                if (interrupts) { // ok, this takes over driving so we should make the
                                                           // trajectory leading up to here.
                    if (waypoints.size() > 0) {
                        actualCommands.add(new TrajectoryFollowPiece(drivetrain, new ArrayList<WaypointPiece>(waypoints), //copy of current list
                                commandPiece.getRequestedStartSpeed(), this));
                        waypoints.clear();
                    }
                    actualCommands.add(commandPiece);
                } else {
                    //TODO make this work on waypoints!!
                    actualCommands.add(new ScheduleCommand(commandPiece));
                }
            }
        }

        for (Command command : actualCommands) {
            group.addCommands(command);
        }
        group.addRequirements((Subsystem)drivetrain);

        return group;
    }

    public DrivetrainConfig getDrivetrainConfig() {
        return drivetrainConfig;
    }

    /***
     * Uses the rotation PID controller to give a turn value needed to maintain
     * heading lock. This is useful when developing your own commands in which you
     * still want rotation lock.
     * 
     * @return the rotation speed in radians per second, ready to be put into a
     *         ChassisSpeeds.
     */
    public double rotationNeededForHeadingLock() {
        rotationController.setConstraints(new TrapezoidProfile.Constraints(drivetrainConfig.maxAnglularVelocity,
                drivetrainConfig.maxAngularAcceleration));
        return rotationController.calculate(drivetrain.getGyroRadians(), headingOffset);
    }

    /**
     * Get the internal rotation controller for this path. Use this controller
     * instead of any custom PID in your commands, because it will be consistent and
     * smooth. If you just need the turn value right now for use, use
     * {@link #rotationNeededForHeadingLock()}
     * 
     * @return the rotation controller
     */
    public ProfiledPIDController getRotationController() {
        return rotationController;
    }

}
