package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class DebugLights extends CommandBase implements CommandPathPiece{
    
    long timeout = 0;
    long timeToStop = 0;
    long period;
    Vision vision;

   
    public DebugLights(Vision vision,long timeout, long period) {

        this.timeout=timeout;
        this.vision= vision;
        this.period = period;

    }
    
    @Override
    public void initialize() {
        timeToStop = System.currentTimeMillis()+timeout;

    }
    
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if(System.currentTimeMillis()%period < period/2){
            vision.setLight(false);
        }else{
            vision.setLight(true);

        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {

        return System.currentTimeMillis()>timeToStop;
    }

    @Override
    public void end(boolean interrupted) {
        vision.setLight(true);
    }
    
    @Override
    public double getRequestedStartSpeed() {
        return 1;
    }

    
}
