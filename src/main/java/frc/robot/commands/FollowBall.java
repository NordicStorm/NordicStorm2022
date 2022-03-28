// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Barrel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PixyObject;

/**
 *
 */
public class FollowBall extends CommandBase{

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    boolean doIntake;
    boolean endWhenClose;
    Drivetrain drivetrain;
    Barrel barrel;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public FollowBall(Drivetrain drivetrain, Barrel barrel, boolean handleIntake, boolean endWhenClose, double forwardMod) {

        this.doIntake = handleIntake;
        this.endWhenClose = endWhenClose;
        this.forwardMod=forwardMod;
        this.drivetrain = drivetrain;
        this.barrel = barrel;
    }

    // Called just before this Command runs the first time
    DriveToObject targetTracker;

    @Override
    public void initialize() {
        

        targetTracker = new DriveToObject(pVal, forwardMod, maxTurn, stopWidth * 0, proxPVal, camWidth, camHeight);
        targetTracker.setOffset(0);
        drivetrain.getPixy().setLamps(true);
        SmartDashboard.putString("currentCommand", "followBall()");

    }

    int camWidth = 315;
    int camHeight = 207;

    double turnValue = 0;
    double forwardValue = 0;
    double maxTurn = 3;
    double pVal = 4.00; // 5
    double proxPVal = 0.07 * 0;
    double stopWidth = 100;
    double forwardMod = 3;
    boolean fullAuto = true;
    boolean hasGotABall=false;
    boolean shouldStop = false;
    double minAspect = 0.8;
    double maxAspect = 3;
    double abortMaxAspect = 6;
    int currentFollowingID = -1;
    long timeToEndDrive = 0;

    private PixyObject findTarget(List<PixyObject> possibleTargets) {
        if (currentFollowingID != -1) {
            for (PixyObject possible : possibleTargets) {
                
                if (possible.trackingIndex == currentFollowingID) {
                    double aspect = ((double)possible.width) / possible.height;
                if(aspect>=abortMaxAspect && possible.y<160){
                    System.out.println("abort because aspect: "+aspect);
                    break; 
                }
                if(possible.width >= 120){
                    System.out.println("Abort because too wide");
                    break;
                }
                    return possible;
                }
            }
            currentFollowingID = -1; // the target was not found in the list, so reset what we're following.
            return findTarget(possibleTargets); // find a new target
        } else {// not locked on
            for (PixyObject possible : possibleTargets) {

                System.out.println(possible);
                if(possible.sig != 1){
                    continue;
                }
                double aspect = ((double)possible.width) / possible.height;
                //System.out.println(aspect);
                if(!(aspect>=minAspect && aspect<=maxAspect)){continue;}//skip if it is too
                // far from square
                currentFollowingID = possible.trackingIndex;
                System.out.println("was chosen!");
                return possible;
            }
        }

        return null;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        
        if (timeToEndDrive < System.currentTimeMillis()) {
            if(hasGotABall && endWhenClose){
                shouldStop=true;//The timer has run out after we have grabbed a ball
            }
            List<PixyObject> objects = drivetrain.getPixy().readObjects();
            PixyObject object = findTarget(objects);
            if (object != null) {
                System.out.println("width:"+object.width);
                System.out.println("y:"+object.y);

                if (object.width > stopWidth && object.y > 170) {// 207 is max/at the bottom of the bot
                    if (endWhenClose) {
                        hasGotABall=true;
                    }
                    timeToEndDrive=System.currentTimeMillis()+300;
                }
            }
            if (object != null) {
                double[] speeds = targetTracker.execute(object.x, object.width);
                turnValue = speeds[0];
                forwardValue = speeds[1];
            } else {
                if (turnValue < 0) {
                    turnValue = -maxTurn * 0.75;
                } else if (turnValue > 0) {
                    turnValue = maxTurn * 0.75;
                }
                //turnValue = 0;
                forwardValue = 0;

            }
            if (Math.abs(turnValue) > 0.1 && Math.abs(turnValue) < 0.15) {
                if (turnValue < 0) {
                    turnValue = -0.15;
                } else {
                    turnValue = 0.15;
                }
            }
        }else{
            forwardValue=2;
            turnValue=0;
            System.out.println("charge!");
        }
        drivetrain.limitDrive(new ChassisSpeeds(forwardValue, turnValue*0, -turnValue), 2);
        if (doIntake) {
            barrel.setIntake(true);
        }

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {

        return shouldStop;
    }

}
