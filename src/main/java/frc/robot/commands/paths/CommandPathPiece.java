package frc.robot.commands.paths;

import edu.wpi.first.wpilibj2.command.*;

public interface CommandPathPiece extends Command, PathPiece {

    /**
     * If the piece will have an effect on the trajectory, so it needs to be
     * interrupted. You should return false if it is an instant instruction, such as setting
     * shooter speed.
     * 
     * @return
     */
    public boolean interruptsTrajectory();

    /**
     * Gets Only needed if interruptsTrajectory is true.
     * 
     * @return the requested speed in meters per second
     */
    default public double getRequestedStartSpeed() {
        return 0;
    }

    @Override
    default PieceType getPieceType() {
        return PieceType.Command;
    }
   
    

}
