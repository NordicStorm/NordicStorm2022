package frc.robot.commands.paths;

import java.lang.reflect.Field;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Used to set the value of a DrivetrainConfig at runtime using reflection
 */
public class ConfigPropertySetPiece extends CommandBase implements CommandPathPiece {
    MultiPartPath path;
    String propName;
    double value;

    public ConfigPropertySetPiece(MultiPartPath path, String propName, double value) {
        this.path = path;
        this.propName = propName;
        this.value = value;
    }

    @Override
    public void initialize() {
        Field field;

        try {
            field = path.drivetrainConfig.getClass().getDeclaredField(propName);
        } catch (NoSuchFieldException | SecurityException e) {
            throw new RuntimeException(e);
        }

        try {
            field.setDouble(path.drivetrain, value);
        } catch (IllegalArgumentException | IllegalAccessException e) {
            throw new RuntimeException(e);
        }
        
        
    }

    
}
