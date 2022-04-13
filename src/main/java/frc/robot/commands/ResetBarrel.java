package frc.robot.commands;

import java.nio.file.Path;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.commands.paths.PathUtil;
import frc.robot.subsystems.Barrel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class ResetBarrel extends CommandBase implements CommandPathPiece{

    public static boolean currentlyRunning = false;
    private Drivetrain drivetrain;
    private Barrel barrel;
    private Vision vision;
    private long timeout;
    private long endingTime;
    private long startTime = 0;
    boolean done = false;
    
  
    /**
     * Take control of the drivetrain and barrel to take the shot
     * 
     * @param drivetrain
     * @param vision
     * @param timeout    in milliseconds
     */
    public ResetBarrel(Drivetrain drivetrain, Barrel barrel, Vision vision, long timeout) {
        this.drivetrain = drivetrain;
        this.barrel = barrel;
        this.vision = vision;
        this.timeout = timeout;

        addRequirements(barrel);

    }
    

    @Override
    public void initialize() {
        this.endingTime = System.currentTimeMillis() + timeout;
        startTime = System.currentTimeMillis();
        done = false;
        barrel.autoBarrel = false;
        timesGood = 0;
        
    }
    int timesGood = 0;
    @Override
    public void execute() {
        long timeGoing = System.currentTimeMillis()-startTime;
        barrel.rawScrew(0.75);
        
        if(Math.abs(barrel.getTiltVelocity())<200 && !(timeGoing<200)) {
            timesGood+=1;
            System.out.println(timesGood);
        }else {
            timesGood = 0;
        }
        if(timesGood>5){
            barrel.resetToPos(78.8); //78.3
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        
        return done || System.currentTimeMillis()>endingTime;
    }
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
    @Override
    public void end(boolean interrupted) {
        barrel.rawScrew(0);
        barrel.autoBarrel =  true;
        barrel.setTiltAngle(barrel.intakePos);
    }
}
