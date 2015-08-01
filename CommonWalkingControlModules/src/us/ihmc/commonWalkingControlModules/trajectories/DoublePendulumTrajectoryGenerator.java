package us.ihmc.commonWalkingControlModules.trajectories;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.simple.SimpleMatrix;
import us.ihmc.robotics.numericalMethods.RungeKuttaSimulation;

import java.util.Arrays;

/**
 * Created by agrabertilton on 2/8/15.
 */
public class DoublePendulumTrajectoryGenerator extends RungeKuttaSimulation
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
   private boolean middleInitialized;

   private double dt = 0.001;
   private double[] timeValues;
   private double[][] positionValues;
   private double[][] velocityValues;
   private double[][] torqueValues;

   private boolean trajectoryComputed = false;

   public DoublePendulumTrajectoryGenerator(double length1, double comRadius1, double mass1, double momentOfInertia1, double length2, double comRadius2, double mass2, double momentOfInertia2){
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

   public void setMidpoint(double middleTime, double[] middlePosition){
      this.middleTime = middleTime;
      this.middlePosition = middlePosition;
      this.middleInitialized = true;
   }

   public void computeTrajectory(){
      if (!startInitialized || !endInitialized) throw new RuntimeException(this.getClass().getSimpleName() + "Not Fully Initialized");

      double timeInterval = endTime - startTime;
      int discretizationNumber = (int) Math.ceil(timeInterval/dt)+1;
      double dtActual = timeInterval / (discretizationNumber - 1);
      timeValues = new double[discretizationNumber];
      for (int i = 0; i < discretizationNumber; i++){
         timeValues[i] = startTime + i*dtActual;
      }

      positionValues = new double[discretizationNumber][2];
      velocityValues = new double[discretizationNumber][2];
      torqueValues = generateTorquesFromAmplitudes(timeValues, 0.7 / timeInterval, 2.0, -.50);

      positionValues[0] = startPosition;
      velocityValues[0] = startVelocity;

      SimpleMatrix currentVariableValues = new SimpleMatrix(startPosition.length + startVelocity.length, 1);

      currentVariableValues.setColumn(0,0, startPosition);
      currentVariableValues.setColumn(0, startPosition.length, startVelocity);


      for (int i = 0; i < discretizationNumber -1; i++)
      {
         SimpleMatrix nextVariableValues = getNextPosition(timeValues[i], currentVariableValues, dtActual);

         double[] variableValues = nextVariableValues.getMatrix().getData();
         positionValues[i + 1] = Arrays.copyOfRange(variableValues, 0, startPosition.length);
         velocityValues[i + 1] = Arrays.copyOfRange(variableValues, startPosition.length, startPosition.length + startVelocity.length);
         currentVariableValues = nextVariableValues;
      }
      trajectoryComputed = true;
   }

   private double[][] generateTorquesFromAmplitudes(double[] timeValues, double timeGain, double baseGain, double baseTorque){
      double[][] torqueValues = new double[timeValues.length][2];
      double[] currentTorques;
      double timeInterval = timeValues[timeValues.length-1] - timeValues[0];
      double percentSwing;
      double x;
      for (int i = 0; i < timeValues.length; i++){
         currentTorques = new double[2];
         percentSwing = (timeValues[i] - timeValues[0])/timeInterval;
         x = percentSwing - 0.5;
         currentTorques[0] = timeGain * baseGain * baseGain * (1.4 * (-8 * Math.pow(x,3) + 3 * x)) + baseTorque;
         torqueValues[i] = currentTorques;
      }
      return torqueValues;
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

   private SimpleMatrix getMassMatrix(double[] jointAngles){
      computeSimplificationTerms();
      DenseMatrix64F massMatrix = new DenseMatrix64F(2,2);
      massMatrix.set(0,0, J0 + J2 + 2*J1*Math.cos(jointAngles[1]));
      massMatrix.set(1,0, J3 + J1*Math.cos(jointAngles[1]));
      massMatrix.set(0,1, J3 + J1*Math.cos(jointAngles[1]));
      massMatrix.set(1,1, J3);
      return SimpleMatrix.wrap(massMatrix);
   }

   private SimpleMatrix getCoriolisTerm(double[] jointAngles, double[] jointVelocities){
      computeSimplificationTerms();
      DenseMatrix64F coriolisMatrix = new DenseMatrix64F(2,1);
      coriolisMatrix.set(0, 0, -J1 * Math.sin(jointAngles[1]) * jointVelocities[1] * (2*jointVelocities[0] + jointAngles[1]));
      coriolisMatrix.set(1, 0, J1 * Math.sin(jointAngles[1]) * Math.pow(jointVelocities[0], 2));
      return SimpleMatrix.wrap(coriolisMatrix);
   }

   private SimpleMatrix getGravitationalTerm(double[] jointAngles){
      computeSimplificationTerms();
      DenseMatrix64F gravitationalMatrix = new DenseMatrix64F(2,1);
      gravitationalMatrix.set(0, 0, G1 * Math.sin(jointAngles[0]) +  G2 * Math.sin(jointAngles[0] + jointAngles[1]));
      gravitationalMatrix.set(1, 0, G2 * Math.sin(jointAngles[0] + jointAngles[1]));
      return SimpleMatrix.wrap(gravitationalMatrix);
   }

   private SimpleMatrix getAccelerationValues(SimpleMatrix jointTorques, SimpleMatrix jointAngles, SimpleMatrix jointVelocities){
      double[] jointAngleArray = jointAngles.getMatrix().getData();
      double[] jointVelocityArray = jointVelocities.getMatrix().getData();

      SimpleMatrix massMatrix = getMassMatrix(jointAngleArray);
      SimpleMatrix coriolisTerm = getCoriolisTerm(jointAngleArray, jointVelocityArray);
      SimpleMatrix gravitationalTerm = getGravitationalTerm(jointAngleArray);

      SimpleMatrix massInverse = massMatrix.invert();
      //A = H^-1 * (T - C - G)
      SimpleMatrix accelerations = massInverse.mult((jointTorques.minus(coriolisTerm)).minus(gravitationalTerm));
      return accelerations;
   }

   public double[] getTimeValues(){
      return timeValues;
   }

   public double[] getPosition(double time){
      double index = getTimeIndex(time);
      int integerIndex = (int) index;
      double interpolationValue = index - integerIndex;
      double[] currentPosition = new double[2];
      if (interpolationValue > 1e-15)
      {
         for (int i = 0; i < positionValues[0].length; i++)
         {
            currentPosition[i] = positionValues[integerIndex][i] + interpolationValue * (positionValues[integerIndex][i + 1] - positionValues[integerIndex][i]);
         }
      }else{
         currentPosition = positionValues[integerIndex];
      }
      return currentPosition;
   }

   public double[] getVelocity(double time){
      double index = getTimeIndex(time);
      int integerIndex = (int) index;
      double interpolationValue = index - integerIndex;
      double[] currentVelocity = new double[2];
      if (interpolationValue > 1e-15)
      {
         for (int i = 0; i < velocityValues[0].length; i++)
         {
            currentVelocity[i] = velocityValues[integerIndex][i] + interpolationValue * (velocityValues[integerIndex][i + 1] - velocityValues[integerIndex][i]);
         }
      }else{
         currentVelocity = velocityValues[integerIndex];
      }
      return currentVelocity;
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

   @Override
   protected SimpleMatrix getDerivativeTerm(double currentTime, SimpleMatrix currentPositionTerm)
   {
      //currentPositionTerm is Position stacked over velocity
      int rowSize = currentPositionTerm.numRows() / 2;
      int columnSize = currentPositionTerm.numCols();

      SimpleMatrix position = currentPositionTerm.extractMatrix(0, rowSize, 0, columnSize);
      SimpleMatrix velocity = currentPositionTerm.extractMatrix(rowSize, 2*rowSize, 0, columnSize);

      SimpleMatrix acceleration = getAccelerationValues(getCurrentTorques(currentTime), position, velocity);
      DenseMatrix64F derivativeTerm = new DenseMatrix64F(currentPositionTerm.getMatrix());
      CommonOps.insert(velocity.getMatrix(), derivativeTerm, 0, 0);
      CommonOps.insert(acceleration.getMatrix(), derivativeTerm, velocity.numRows(), 0);
      return SimpleMatrix.wrap(derivativeTerm);
   }

   private SimpleMatrix getCurrentTorques(double time){
      int startIndex = 0;
      for (int i = 0; i < timeValues.length; i++){
         if (time > timeValues[i]){
            startIndex = i;
         }
      }
      double[] torques = Arrays.copyOf(torqueValues[startIndex], torqueValues[startIndex].length);
      if (startIndex != timeValues.length -1){
         double interpolationValue = (timeValues[startIndex+1] - time) / (timeValues[startIndex+1] - timeValues[startIndex]);
         for (int i = 0; i < torques.length; i++){
            torques[i] += interpolationValue * torqueValues[startIndex+1][i];
         }
      }
      return new SimpleMatrix(torques.length, 1, false, torques);
   }
}
