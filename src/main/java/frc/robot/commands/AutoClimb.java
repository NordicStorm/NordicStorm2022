package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climbers;

public class AutoClimb extends CommandBase{

    enum ClimbState{
        BottomClipped,
        TopDetached

        
    }
    Climbers climbers;
    double d;

    public AutoClimb(Climbers climbers){
        this.climbers = climbers;
    }
    @Override
    public void initialize() {
        
    }


    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }
}
