package frc.robot.commands.paths;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PathUtil {
    /**
     * Convert the ChassisSpeeds, which has x and y components, into a single speed number using the pythagorean theorem.
     * @param speeds
     * @return
     */
    public static double linearSpeedFromChassisSpeeds(ChassisSpeeds speeds) {
        return Math.sqrt(speeds.vxMetersPerSecond * speeds.vxMetersPerSecond
                + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);
    }

    
}
