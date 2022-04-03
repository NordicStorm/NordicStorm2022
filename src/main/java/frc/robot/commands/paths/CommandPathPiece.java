package frc.robot.commands.paths;

import edu.wpi.first.wpilibj2.command.*;

public interface CommandPathPiece extends Command, PathPiece {
    
    /**
     * Is used only needed if interruptsTrajectory is true.
     * 
     * @return the requested speed in meters per second
     */
    default public double getRequestedStartSpeed() {
        return 0;
    }
    boolean interrupts = false;

    @Override
    default PieceType getPieceType() {
        return PieceType.Command;
    }
   
    

}
