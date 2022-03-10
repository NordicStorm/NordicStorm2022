package frc.robot.commands.paths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class takes any standard Command and makes into a CommandPathPiece. This
 * class shouldn't normally be used by the user, please make your command
 * implement CommandPathPiece instead.
 */
public class PathPieceWrapper extends SequentialCommandGroup implements CommandPathPiece {

    private double startSpeed;
    private boolean interruptsTrajectory;
    public PathPieceWrapper(Command command, boolean interruptsTrajectory, double startSpeed) {
        this.startSpeed = startSpeed;
        this.interruptsTrajectory = interruptsTrajectory;
        addCommands(command);
    }

    @Override
    public boolean interruptsTrajectory() {
        return interruptsTrajectory;
    }

    @Override
    public double getRequestedStartSpeed() {
        return startSpeed;
    }

}
