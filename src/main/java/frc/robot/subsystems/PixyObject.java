package frc.robot.subsystems;

public class PixyObject {
    public int x;
    public int y;
    public int sig;
    public int width;
    public int height;
    public int trackingIndex;
    public int age;
	public int angle;

    public String toString(){
        return "Object type "+sig+" at ("+x+","+y+") w="+width+", h="+height+". Index="+trackingIndex+", age="+age+", angle="+angle;
    }
    public int getSum(){
        return x+y+sig+width+height+trackingIndex+age+angle;
    }
}