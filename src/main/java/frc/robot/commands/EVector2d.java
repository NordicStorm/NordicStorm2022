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

    public EVector2d normalized(){
        var other = new EVector2d(x, y);
        other.normalize();
        return other;
    }

    public EVector2d times(double factor){
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
    public void setMagnitude(double mag){
        normalize();
        multiply(mag);
    }
    /**
     * returns new vector of this - other
     */
    public EVector2d minus(EVector2d other){
        return new EVector2d(x-other.x, y-other.y);
    }

    @Override
    public String toString() {
        return "("+x+", "+y+")";
    }
    
}
