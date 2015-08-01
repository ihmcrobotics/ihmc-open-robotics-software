package us.ihmc.commonWalkingControlModules.trajectories;

import org.ejml.simple.SimpleMatrix;
import us.ihmc.robotics.numericalMethods.RungeKuttaSimulation;
import us.ihmc.robotics.trajectories.ParametricSplineTrajectory;
import us.ihmc.robotics.trajectories.ParametricSplineTrajectorySolver;

import java.util.Arrays;

/**
 * Created by agrabertilton on 2/10/15.
 */
public class HybridDoublePendulumTrajectoryGenerator extends RungeKuttaSimulation
{
   double length1 = 1.0;
   double comRadius1 = 0.5;
   double mass1 = 1.0;
   double momentOfInertia1 = 0.0;

   double length2 = 1.0;
   double comRadius2 = 0.5;
   double mass2 = 1.0;
   double momentOfInertia2 = 0.0;

   private double J0;
   private double J1;
   private double J2;
   private double J3;
   private double G1;
   private double G2;
   private boolean termsComputed = false;

   private double startTime;
   private double[] startPosition;
   private double[] startVelocity;
   private boolean startInitialized;

   private double endTime;
   private double[] endPosition;
   private double[] endVelocity;
   private boolean endInitialized;

   private double middleTime;
   private double[] middlePosition;
   private double[] middleVelocity;
   private boolean middleInitialized;

   double swingTime;
   double timeGain;
   double baseGain;
   double baseTorque;

   private double dt = 0.001;
   private double[] timeValues;
   ParametricSplineTrajectory kneeTrajectory;
   private double[] hipPositionValues;
   private double[] hipVelocityValues;

   private boolean trajectoryComputed = false;

   public HybridDoublePendulumTrajectoryGenerator(double length1, double comRadius1, double mass1, double momentOfInertia1, double length2, double comRadius2, double mass2, double momentOfInertia2){
      this.length1 = length1;
      this.comRadius1 = comRadius1;
      this.mass1 = mass1;
      this.momentOfInertia1 = momentOfInertia1;
      this.length2 = length2;
      this.comRadius2 = comRadius2;
      this.mass2 = mass2;
      this.momentOfInertia2 = momentOfInertia2;
   }

   public void setStartValues(double startTime, double[] startPosition, double[] startVelocity){
      this.startTime = startTime;
      this.startPosition = startPosition;
      this.startVelocity = startVelocity;
      this.startInitialized = true;
   }

   public void setEndValues(double endTime, double[] endPosition, double[] endVelocity){
      this.endTime = endTime;
      this.endPosition = endPosition;
      this.endVelocity = endVelocity;
      this.endInitialized = true;
   }

   public void setMidpoint(double middleTime, double[] middlePosition, double[] middleVelocity){
      this.middleTime = middleTime;
      this.middlePosition = middlePosition;
      this.middleVelocity = middleVelocity;
      this.middleInitialized = true;
   }

   public void computeTrajectory(){
      if (!startInitialized || !endInitialized || !middleInitialized) throw new RuntimeException(this.getClass().getSimpleName() + "Not Fully Initialized");

      double timeInterval = endTime - startTime;
      int discretizationNumber = (int) Math.ceil(timeInterval/dt)+1;
      double dtActual = timeInterval / (discretizationNumber - 1);
      timeValues = new double[discretizationNumber];
      for (int i = 0; i < discretizationNumber; i++){
         timeValues[i] = startTime + i*dtActual;
      }

      ParametricSplineTrajectorySolver kneeSolver = new ParametricSplineTrajectorySolver(3,3, 6, 1);
      double[] times = new double[kneeSolver.getNumberOfSplines() + 1];
      for (int i = 0; i < times.length; i++){
         times[i] = Double.NaN;
      }
      times[0] = startTime;
      times[1] = startTime + (endTime - startTime) * 0.25;
      times[2] = startTime + (endTime - startTime) * 0.75;
      times[times.length-1] = endTime;
      kneeSolver.setTimes(times);

      kneeSolver.setPositionConstraint(startTime, Arrays.copyOfRange(startPosition, 1,2));
      kneeSolver.setVelocityConstraint(startTime, Arrays.copyOfRange(startVelocity, 1, 2));
      kneeSolver.setPositionConstraint(endTime, Arrays.copyOfRange(endPosition, 1, 2));
      kneeSolver.setVelocityConstraint(endTime, Arrays.copyOfRange(endVelocity, 1, 2));
      kneeSolver.setPositionConstraint(middleTime, Arrays.copyOfRange(middlePosition, 1,2));
      kneeSolver.setVelocityConstraint(middleTime, Arrays.copyOfRange(middleVelocity, 1,2));
      kneeTrajectory = kneeSolver.computeTrajectory();

      swingTime = endTime - startTime;
      timeGain = 1.0;
      baseGain = .5;
      baseTorque = 0;

      SimpleMatrix currentHipPV = new SimpleMatrix(2,1);
      currentHipPV.set(0,0, startPosition[0]);
      currentHipPV.set(1,0, startVelocity[0]);

      hipPositionValues = new double[discretizationNumber];
      hipVelocityValues = new double[discretizationNumber];

      for (int i = 0; i < discretizationNumber -1; i++)
      {
         SimpleMatrix nextHipPV = getNextPosition(timeValues[i], currentHipPV, dtActual);
         hipPositionValues[i+1] = nextHipPV.get(0);
         hipVelocityValues[i+1] = nextHipPV.get(1);
         currentHipPV = nextHipPV;
      }
      trajectoryComputed = true;

      double endHipPositionError = endPosition[0] - hipPositionValues[discretizationNumber -1];
      double endHipVelocityError = endVelocity[0] - hipVelocityValues[discretizationNumber -1];
      System.out.println("Hip Position Error is: " + endHipPositionError);
      System.out.println("Hip Velocity Error is: " + endHipVelocityError);
   }

   private void computeSimplificationTerms(){
      if (termsComputed) return;
      J0 = mass1 * Math.pow(comRadius1, 2) + momentOfInertia1;
      J1 = mass2 * length1 * comRadius2;
      J2 = mass2 * (Math.pow(length1, 2) + Math.pow(comRadius2, 2)) + momentOfInertia2;
      J3 = mass2 * Math.pow(comRadius2, 2) + momentOfInertia2;
      G1 = (mass1 * comRadius1 + mass2 * length1) * (9.81);
      G2 = (mass2 * comRadius2) * (9.81);
      termsComputed = true;
   }

   @Override
   protected SimpleMatrix getDerivativeTerm(double currentTime, SimpleMatrix currentHipPV)
   {
      computeSimplificationTerms();
      double kneePosition = kneeTrajectory.getPositions(currentTime)[0];
      double kneeVelocity = kneeTrajectory.getVelocities(currentTime)[0];
      double kneeAcceleration = kneeTrajectory.getAccelerations(currentTime)[0];

      double hipPosition = currentHipPV.get(0,0);
      double hipVelocity = currentHipPV.get(1,0);

      double denominator = J0 + J2 + 2*J1* Math.cos(kneePosition);
      double torque = getTorque(currentTime);
      double kneeAccelerationTerm = -1 * (J3 + J1*Math.cos(kneePosition)) * kneeAcceleration;
      double velocityTerm = J1 * Math.sin(kneePosition) * kneeVelocity * (2 * hipVelocity + kneeVelocity);
      double gravityTerm = -1 * G1 *Math.sin(hipPosition) - G2 * Math.sin(hipPosition + kneePosition);

      double hipAcceleration = (torque + kneeAccelerationTerm + velocityTerm + gravityTerm) / denominator;

      SimpleMatrix nextPosition = new SimpleMatrix(currentHipPV);
      nextPosition.set(0, 0, hipVelocity);
      nextPosition.set(1, 0, hipAcceleration);
      return nextPosition;
   }

   private double getTorque(double time){
      double percentSwing = time/swingTime;
      double x = percentSwing - 0.5;
      double currentTorque = timeGain * baseGain * (1.4 * (-8 * Math.pow(x,3) + 3 * x)) + baseTorque;
      return currentTorque;
   }

   public double[] getTimeValues(){
      return timeValues;
   }

   public double[] getPosition(double time){
      double index = getTimeIndex(time);
      int integerIndex = (int) index;

      double[] position = new double[2];
      position[0] = hipPositionValues[integerIndex];
      if (integerIndex != hipPositionValues.length -1){
         position[0] += (index - integerIndex) * hipPositionValues[integerIndex+1];
      }

      position[1] = kneeTrajectory.getPositions(time)[0];
      return position;
   }

   public double[] getVelocity(double time){
      double index = getTimeIndex(time);
      int integerIndex = (int) index;

      double[] velocity = new double[2];
      velocity[0] = hipVelocityValues[integerIndex];
      if (integerIndex != hipVelocityValues.length -1){
         velocity[0] += (index - integerIndex) * hipVelocityValues[integerIndex+1];
      }

      velocity[1] = kneeTrajectory.getVelocities(time)[0];
      return velocity;
   }

   public double[] getAcceleration(double time){
      double[] acceleration = new double[2];
      SimpleMatrix currentHipPV = new SimpleMatrix(2,1);
      currentHipPV.set(0,0, getPosition(time)[0]);
      currentHipPV.set(1,0, getVelocity(time)[0]);
      acceleration[0] = getDerivativeTerm(time, currentHipPV).get(1,0);
      acceleration[1] = kneeTrajectory.getAccelerations(time)[0];
      return acceleration;
   }

   private double getTimeIndex(double time){
      if (!timeValid(time)){
         throw new RuntimeException(this.getClass().getSimpleName() + " Invalid Time Input");
      }
      int startIndex = 0;
      for (int i = 0; i < timeValues.length; i++){
         if (time >= timeValues[i])
         {
            startIndex = i;
         }
         else
         {
            break;
         }
      }
      if (startIndex == timeValues.length-1){
         return (double) startIndex;
      }
      double interpolationValue = (timeValues[startIndex+1] - time)/(timeValues[startIndex+1] - timeValues[startIndex]);
      return startIndex+interpolationValue;
   }

   private boolean timeValid(double time){
      return (time >= timeValues[0] && time <= timeValues[timeValues.length-1]);
   }

}
