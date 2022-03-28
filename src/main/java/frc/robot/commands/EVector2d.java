package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.Vector2d;

/**
 * Extended Vector2d with some extra utility features
 */
public class EVector2d extends Vector2d{

    public EVector2d(double x, double y) {
        super(x, y);
    }
    /**
     * normalize the vector in place
     * */
    public void normalize(){
        double d = magnitude();
        x/=d;
        y/=d;
    }

    public EVector2d multipliedBy(double factor){
        return new EVector2d(x*factor, y*factor);
    }
    /**
     * Multiplies in place
     * @param factor
     */
    public void multiply(double factor){
        x*=factor;
        y*=factor; 
    }
    /**
     * returns new vector of this - other
     */
    public EVector2d minus(EVector2d other){
        return new EVector2d(x-other.x, y*other.y);
    }
    
}
