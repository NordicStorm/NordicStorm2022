package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climbers;

public class AutoClimb extends CommandBase{

    enum ClimbState{
        ExtendingWhileClear,
        ExtendingFullUp,
        ClippingDown,
        WaitingToPull,
        UnhookingBottom,
        WaitingForSprings,
        ChinningUp,
        WaitingToUnclip,
        UnclippingAxes

        
    }
    Climbers climbers;
    

    public AutoClimb(Climbers climbers){
        this.climbers = climbers;
    }
    @Override
    public void initialize() {
        
    }


    @Override
    public void execute() {
        double currentPos = 
    }

    private double getAppropiatePower(double currentPos, double currentSpeed, double currentTilt){
        return 0;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }
}
