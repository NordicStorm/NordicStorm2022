package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;

public class AutoClimb extends CommandBase {

    enum ClimbState {
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

    double clearPos = 33.8;
    double topPos = 49.5;
    double clipPos = 39;
    double underPos = 30;
    double unhookPos = 20;
    double waitPos = 20;
    double chinPos = 3;
    double unclipPos = 10;

    double pushable = 43;
    double clipable = 47;
    double failClip = 40;

    public AutoClimb(Climbers climbers, Drivetrain drivetrain) {
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
        double leftPower = getAppropiatePower(climbers.getLeftPos(), climbers.getLeftSpeed(), currentTilt, true);
        double rightPower = getAppropiatePower(climbers.getRightPos(), climbers.getRightSpeed(), currentTilt, false);

    }

    private double getAppropiatePower(double currentPos, double currentSpeed, double pitch, boolean left) {
        double power = 0;
        if (currentState == ClimbState.ExtendingWhileClear) {
            power = 1;
            if(currentPos>=clearPos){
                power = 0;
                requestStateChange(ClimbState.ExtendingFullUp, left, false);
            }
        } else if (currentState == ClimbState.ExtendingFullUp) {
            power = 0.5;
            if(pitch<pushable){
                power = 0;
            }
            if(currentPos>=topPos){
                requestStateChange(ClimbState.ClippingDown, left, false);
            }
        } else if (currentState == ClimbState.ClippingDown) {

        } else if (currentState == ClimbState.WaitingToPull) {

        } else if (currentState == ClimbState.UnhookingBottom) {

        } else if (currentState == ClimbState.WaitingForSprings) {

        } else if (currentState == ClimbState.ChinningUp) {

        } else if (currentState == ClimbState.WaitingToUnclip) {

        } else if (currentState == ClimbState.UnclippingAxes) {

        }
        return power;
    }

    private ClimbState requestedLeft = ClimbState.ChinningUp;
    private ClimbState requestedRight = ClimbState.ChinningUp;

    /**
     * Use this when one climber is ready to change state
     * 
     * @param newState
     * @param left
     * @param isAbort
     */
    private void requestStateChange(ClimbState newState, boolean left, boolean isAbort) {
        if (isAbort) {
            currentState = newState;
        } else {
            if (left) {
                requestedLeft = newState;
            } else {
                requestedRight = newState;
            }
            if (requestedLeft == requestedRight) {
                currentState = newState;
            }
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
