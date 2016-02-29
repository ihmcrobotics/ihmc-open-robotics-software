package us.ihmc.robotics.trajectories;

import java.util.Arrays;

/**
 * Created by agrabertilton on 2/4/15.
 */
public class ParametricSplineTrajectory
{
   int numberOfSplines;
   int numberOfVariables;
   double [] times;
   boolean timesInitialized = false;

   double [][][] splineCoefficients; //spline number, coefficients, variables
   boolean[] splinesInitialized;


   public ParametricSplineTrajectory(int numberOfSplines, int numberOfVariables){
      this.numberOfSplines = numberOfSplines;
      this.numberOfVariables = numberOfVariables;
      times = new double[numberOfSplines + 1];
      splinesInitialized = new boolean[numberOfSplines];
      splineCoefficients = new double[numberOfSplines][4][numberOfVariables];
   }

   public void setTimes(double[] newTimes){
      boolean zeroStartTime = false;

      if (newTimes.length != times.length){ //check length
         if (newTimes.length == times.length - 1){
            zeroStartTime = true;
         }else{
            throw new RuntimeException(this.getClass().getSimpleName() + ": Invalid Time Inputs");
         }
      }

      for (int i = 1; i < newTimes.length; i++){
         if (newTimes[i] <= newTimes[i-1] || newTimes[i] < 0.0){
            throw new RuntimeException(this.getClass().getSimpleName() + ": Invalid Time Inputs");
         }
      }

      int index = 0;
      if (zeroStartTime){
         times[0] = 0.0;
         index ++;
      }

      for (double time: newTimes){
         times[index] = time;
         index++;
      }
      timesInitialized = true;
   }

   /**
    *
    * @param splineIndex index of the spline to set
    * @param splineCoefficients the coefficients of the spline, highest order first i.e. Ax + B, where A and B are vectors corresponding to the different variables
    */
   public void setSplineCoefficients(int splineIndex, double[][] splineCoefficients)
   {
      if (splineIndex < 0 || splineIndex >= this.splineCoefficients.length)
         throw new RuntimeException(this.getClass().getSimpleName() + ": Invalid Spline Number");

      for (int i = 0; i < splineCoefficients.length; i++){
         if (splineCoefficients[i].length != numberOfVariables){
            throw new RuntimeException(this.getClass().getSimpleName() + ": Invalid Coefficient Inputs");
         }
      }

      for (int i = 0; i < splineCoefficients.length; i++){
         this.splineCoefficients[splineIndex][i] = Arrays.copyOf(splineCoefficients[i], splineCoefficients[i].length);
      }
      splinesInitialized[splineIndex] = true;
   }

   private boolean hasBeenInitialized(){
      if (!timesInitialized) return false;
      for (boolean initialized : splinesInitialized){
         if (!initialized) return false;
      }
      return true;
   }

   public double[] getPositions(double time){
      return getDerivativeValues(0, time);
   }

   public double[] getVelocities(double time){
      return getDerivativeValues(1, time);
   }

   public double[] getAccelerations(double time){
      return getDerivativeValues(2,time);
   }

   public double[] getDerivativeValues(int derivativeNumber, double time){
      if (!hasBeenInitialized()){
         throw new RuntimeException(this.getClass().getSimpleName() + ": Has Not Been Initialized");
      }

      int relevantSplineIndex = getRelevantSpline(time);
      double[][] relevantSpline = splineCoefficients[relevantSplineIndex];
      int orderOfSpline = relevantSpline.length - 1;

      int[] powerValue = new int[orderOfSpline + 1];
      int[] derivativeCoefficient = new int[orderOfSpline + 1];
      for (int i = 0; i <= orderOfSpline; i++){
         powerValue[i] = orderOfSpline - i;
         derivativeCoefficient[i] = 1;
      }
      for (int i = 0; i < derivativeNumber; i++){
         for (int j = 0; j <= orderOfSpline; j++){
            derivativeCoefficient[j] *= powerValue[j];
            powerValue[j] = Math.max(0, powerValue[j] - 1);
         }
      }

      double values[] = new double[numberOfVariables];
      for (int i = 0; i < numberOfVariables; i++){
         values[i] = 0;
         for (int j = 0; j <= orderOfSpline; j++){
            values[i] += relevantSpline[j][i] * derivativeCoefficient[j] * Math.pow(time, powerValue[j]);
         }
      }
      return values;
   }

   private int getRelevantSpline(double time){
      int relevantSpline = 0;
      if (time < times[0] || time > times[times.length-1]){
         throw new RuntimeException(this.getClass().getSimpleName() + ": Invalid Time For Computation");
      }

      for (int i = 0; i < times.length; i++){
         if (time > times[i])
            relevantSpline = i;
      }
      return relevantSpline;
   }

}

