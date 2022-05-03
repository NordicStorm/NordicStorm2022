package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;

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
    ClimbState currentState;
    Climbers climbers;
    Drivetrain drivetrain;

    double clearPos=33.8;
    double topPos=49.5;
    double clipPos=39;
    double underPos=30;
    double unhookPos=20;
    double waitPos=20;
    double chinPos=3;
    double unclipPos=10;
    
    double pushable =43;
    double clipable =47;
    double failClip =40;
    public AutoClimb(Climbers climbers, Drivetrain drivetrain){
        this.climbers = climbers;
        this.drivetrain = drivetrain;
    }
    @Override
    public void initialize() {
        currentState = ClimbState.ChinningUp;
    }


    @Override
    public void execute() {
        double currentTilt = drivetrain.getGyroPitch();
        double leftPower = getAppropiatePower(climbers.getLeftPos(), climbers.getLeftSpeed(), currentTilt);
        double rightPower = getAppropiatePower(climbers.getLeftPos(), climbers.getLeftSpeed(), currentTilt);

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
