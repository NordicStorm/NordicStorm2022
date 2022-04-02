package frc.robot.commands;

public class RollingAverage {
    private final double[] samples;
    private final int numSamples;
    private int currentIndex;
    private int currentContainedSamples = 0;
    /**
     * Class for smoothing out samples
     * @param numSamples the number to keep
     */
    public RollingAverage(int numSamples){
        samples = new double[numSamples];
        this.numSamples = numSamples;
    }

    /**
     * Add a measurement to the average
     * @param sample
     */
    public void put(double sample){
        samples[currentIndex] = sample;
        currentIndex += 1;
        if(currentContainedSamples < currentIndex){
            currentContainedSamples = currentIndex;
        }
        if(currentIndex>=numSamples){
            currentIndex = 0;
        }
    }
    /**
     * Get the average over the past n samples
     * @return
     */
    public double get(){
        double res = 0;
        for(int i = 0; i<currentContainedSamples; ++i){
            res+=samples[i];
        }
        return res/currentContainedSamples;
    }

    public void clear(){
        currentContainedSamples = 1; //prevent a division by 0
        currentIndex = 0;
        samples[0]=0;
    }
}
