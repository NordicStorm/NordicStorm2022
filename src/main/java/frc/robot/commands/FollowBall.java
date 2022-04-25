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

import java.util.List;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.paths.CommandPathPiece;
import frc.robot.subsystems.Barrel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PixyObject;

/**
 *
 */
public class FollowBall extends CommandBase implements CommandPathPiece{


    int camWidth = 315;
    int camHeight = 207;

    double turnValue = 0;
    double forwardValue = 0;
    double maxTurn = 3;
    double pVal = 4.00; // 5
    double proxPVal = 0.07 * 0;
    double stopWidth = 80;
    double forwardMod = 2;
    boolean fullAuto = true;
    boolean hasGotABall = false;
    boolean shouldStop = false;
    static double minAspect = 0.8;
    static double maxAspect = 2;
    static double abortMaxAspect = 3;
    int currentFollowingID = -1;
    long timeToEndDrive = 0;
    int widthMetFor = -1; // this takes an ID if the ball becomes wide enough to be chargeworthy


    long chargeTime = 0;

    boolean doIntake;
    boolean endWhenClose;
    Drivetrain drivetrain;
    Barrel barrel;
    int targetColor = 0;
    boolean canAbort = false;
    double chargeSpeed = 0;
    public FollowBall(Drivetrain drivetrain, Barrel barrel, boolean handleIntake, boolean endWhenClose,
            double forwardMod, int targetColor, double chargeSpeed, boolean canAbort, long chargeTime) {

        this.targetColor = targetColor;
        this.doIntake = handleIntake;
        this.endWhenClose = endWhenClose;
        this.forwardMod = forwardMod;
        this.drivetrain = drivetrain;
        this.barrel = barrel;
        this.chargeSpeed = chargeSpeed;
        this.canAbort = canAbort;
        //chargeTime = (long) ((2/chargeSpeed)*300);
        this.chargeTime = chargeTime;
        addRequirements(barrel);

    }
    public FollowBall(Drivetrain drivetrain, Barrel barrel, boolean handleIntake, boolean endWhenClose,
    double forwardMod, int targetColor, double chargeSpeed, long chargeTime) {
        this(drivetrain, barrel, handleIntake, endWhenClose,
        forwardMod, targetColor, chargeSpeed, false, chargeTime);
    }
    // Called just before this Command runs the first time
    DriveToObject targetTracker;

    @Override
    public void initialize() {
        targetColor = drivetrain.myBallColor;
        targetTracker = new DriveToObject(pVal, forwardMod, maxTurn, stopWidth * 0, proxPVal, camWidth, camHeight);
        targetTracker.setOffset(0);
        drivetrain.getPixy().setLamps(true);

    }


    /**
     * 
     * @param possible
     * @param alreadyTracking
     * @param targetColor 1 means blue, 2 means red
     * @return
     */
    public static boolean isValidBall(PixyObject possible, boolean alreadyTracking, int targetColor) {
        //System.out.println(possible);
        
        double aspect = ((double) possible.width) / possible.height;
        if(possible.y<50){
            return false;
        }
        if (alreadyTracking) {
            if (aspect >= abortMaxAspect && possible.y < 170) {
                System.out.println("abort because aspect: " + aspect);
                return false;
            }
        }else{
            if (possible.sig != targetColor) {
                return false;
            }
            if (!(aspect >= minAspect && aspect <= maxAspect)) {
                return false;
            } // skip if it is too
              // far from square
        }

        double y = possible.y;
        double maxWidth = 0;
        if (possible.width >= 200) {
            //System.out.println("Abort because too wide");
            return false;
        }
        return true;

    }
    public static int countTargets(List<PixyObject> possibleTargets, int color){
        int count = 0;
        for(PixyObject o : possibleTargets){
            if(isValidBall(o, false, color)){
                ++count;
            }
        }
        return count;
    }
    private PixyObject findTarget(List<PixyObject> possibleTargets) {
        if (currentFollowingID != -1) {
            for (PixyObject possible : possibleTargets) {
                if (possible.trackingIndex == currentFollowingID) {
                    if (widthMetFor!=possible.trackingIndex && canAbort && !isValidBall(possible, true, targetColor)) {
                        System.out.println("brek");
                        break;
                    }
                    return possible;
                }
                
            }

            currentFollowingID = -1; // the target was not found in the list, so reset what we're following.
            return findTarget(possibleTargets); // find a new target
        } else {// not locked on
            for (PixyObject possible : possibleTargets) {

                //System.out.println(possible);
                if(isValidBall(possible, false, targetColor)){
                    currentFollowingID = possible.trackingIndex;
                    //System.out.println("was chosen!");
                    return possible;
                }
                
               
            }
        }

        return null;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        barrel.setTiltAngle(barrel.intakePos);

        if (timeToEndDrive < System.currentTimeMillis()) {
            if (turnValue == 0) {
                turnValue = 0.2;
            }
            if (hasGotABall && endWhenClose) {
                shouldStop = true;// The timer has run out after we have grabbed a ball
                System.out.println("done");

            }
            List<PixyObject> objects = drivetrain.getPixy().readObjects();
            PixyObject object = findTarget(objects);
            if (object != null) {
                System.out.println("width:" + object.width);
                System.out.println("height:" + object.height);
                System.out.println("y:" + object.y);

                if (object.width > stopWidth && object.y+object.height >= 190) {// 207 is max/at the bottom of the bot
                    widthMetFor = object.trackingIndex;
                    System.out.println("widthmet!");

                }else{
                    //widthMetFor = -1;
                }
                if(widthMetFor == object.trackingIndex && object.y>195){
                    if (endWhenClose) {
                        hasGotABall = true;
                    }
                    System.out.println("charge!");

                    timeToEndDrive = System.currentTimeMillis() + chargeTime;
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
                // turnValue = 0;
                forwardValue = 0;
                widthMetFor = -1;

            }
            if (Math.abs(turnValue) > 0.1 && Math.abs(turnValue) < 0.15) {
                if (turnValue < 0) {
                    turnValue = -0.15;
                } else {
                    turnValue = 0.15;
                }
            }
        } else {
            forwardValue = chargeSpeed;
            turnValue = 0;
        }
        if(barrel.hasBottomBall()){
            forwardValue = 0;
        }
        drivetrain.limitDrive(new ChassisSpeeds(forwardValue, -turnValue * 0.1, -turnValue), 2);
        if (doIntake) {
            barrel.setIntake(true);
        }

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {

        return shouldStop;// || barrel.hasBottomBall();
    }

    
    @Override
    public double getRequestedStartSpeed() {
        return 1;
    }

}
