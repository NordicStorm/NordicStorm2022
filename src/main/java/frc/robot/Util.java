package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.swervedrivespecialties.swervelib.SwerveModule;

public class Util {
    public static SwerveModuleState stateFromModule(SwerveModule module){
        return new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle()));
    }
    public static ChassisSpeeds rotateSpeeds(ChassisSpeeds speeds, double degrees){
        return ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, Rotation2d.fromDegrees(degrees));
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    public static double applyDeadzone(double value, double deadzone){
        if(Math.abs(value)<deadzone){
            return 0;
        }else{
            return Math.copySign(map(Math.abs(value), deadzone, 1, 0, 1), value);
        }
    }
    public static double absClamp(double val, double max){
        if(val<-max){
            return -max;
        }
        if(val>max){
            return max;
        }
        return val;
    }
    public static double clamp(double n, double min, double max){
        return Math.max(Math.min(n, max), min);
    }
    public static double minWithAbs(double a, double b){
        return Math.abs(a) < Math.abs(b) ? a : b;
    }
    public static double maxWithAbs(double a, double b){
        return Math.abs(a) > Math.abs(b) ? a : b;
    }
    public static double signedSquare(double x) { 
        if(x<0) return -x*x; 
        else return x*x; 
    }
    /**
     * Positive is CCW. Return radians
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     */
    public static double angleBetweenPoints(double x1, double y1, double x2, double y2){
        return Math.atan2(y2-y1, x2-x1);
    }

    public static double angleBetweenPoses(Pose2d start, Pose2d end){
        return angleBetweenPoints(start.getX(), start.getY(), end.getX(), end.getY());
    }
    /**
     *  
     * @param current degrees!
     * @param target
     * @return degrees of difference. Positive means that the target is greater than the current
     */
    public static double angleDiff(double current, double target) {
        current = current % 360 - 180;
        target = target % 360 - 180;
        double diff = (target - current);
        if (diff > 180) {
            diff = -360 + diff;
        }
        if (diff < -180) {
            diff = 360 + diff;
        }
        return diff;
    }
}
