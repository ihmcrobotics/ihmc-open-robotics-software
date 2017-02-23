package us.ihmc.robotics.trajectories;

import java.util.Arrays;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

/**
 * Created by agrabertilton on 2/10/15.
 */
public class ParametricSplineTrajectorySolver
{
   static final boolean DEBUG = false;

   int orderOfSplines;
   int continuityConstraints;
   int numberOfParametersPerSpline;
   int numberOfSplines;
   int numberOfVariables;
   int numberOfConstraints;
   int numberOfConstraintsSet = 0;
   int currentRow = 0;

   double[] times;
   boolean timesSet;

   //constraintMatrix * ParameterVector = constraintVector;
   DenseMatrix64F constraintMatrix;
   DenseMatrix64F constraintVector;

   /**
    *
    * @param orderOfSplines the order of the spline, 1 for linear, 2 for quadratic, 3 for cubic, etc.
    * @param continuityConstraintNumber the number of continuity constraints, 1 = continuous position, 2 = continuous velocity, 3 acceleration, etc.
    * @param numberOfConstraints the number of constraints to be set, needed to determine the size of the matrices
    * @param numberOfVariables the number of variables for the parametric equations, 2D = 2, 3D = 3, JointSpace = numberOfJoints, etc.
    */
   public ParametricSplineTrajectorySolver(int orderOfSplines, int continuityConstraintNumber, int numberOfConstraints, int numberOfVariables){
      this.numberOfConstraints = numberOfConstraints;
      if (continuityConstraintNumber > orderOfSplines) throw new RuntimeException(this.getClass().getSimpleName() + ": Invalid Order vs Continuity Constraints");

      this.orderOfSplines = orderOfSplines;
      numberOfParametersPerSpline = orderOfSplines+1;
      continuityConstraints = continuityConstraintNumber;

      double prospectiveNumber = (numberOfConstraints - continuityConstraints) / (numberOfParametersPerSpline - continuityConstraints);
      if (prospectiveNumber % 1 > 1e-15 || prospectiveNumber < 0) throw new RuntimeException(this.getClass().getSimpleName() + ": Invalid Initialization of Solver");

      numberOfSplines = (int) prospectiveNumber;
      times = new double[numberOfSplines + 1];
      this.numberOfVariables = numberOfVariables;
      int matrixLengths = numberOfParametersPerSpline * numberOfSplines * numberOfVariables;
      constraintMatrix = new DenseMatrix64F(matrixLengths, matrixLengths);
      constraintVector = new DenseMatrix64F(matrixLengths, 1);
   }

   public ParametricSplineTrajectory computeTrajectory(){
      if (numberOfConstraintsSet != numberOfConstraints) throw new RuntimeException(this.getClass().getSimpleName() + ": Not Enough Constraints");
      //set all the continuity constraints
      //Continuity Constraints
      for (int i = 1; i < times.length - 1; i++){
         for (int j = 0; j < continuityConstraints; j++){
            setContinuityConstraint(i, j, times[i]);
         }
      }
      if (CommonOps.det(constraintMatrix) == 0.0){
         if (DEBUG) System.out.println(constraintMatrix);
         throw new RuntimeException(this.getClass().getSimpleName() + ": Invalid Constraints");
      }

      DenseMatrix64F parameterVector = new DenseMatrix64F(numberOfSplines * numberOfParametersPerSpline * numberOfVariables, 1);
      if (!CommonOps.solve(constraintMatrix, constraintVector, parameterVector)){
         throw new RuntimeException(this.getClass().getSimpleName() + ": Could not solve for parameters");
      }

      ParametricSplineTrajectory toReturn = new ParametricSplineTrajectory(numberOfSplines, numberOfVariables);
      toReturn.setTimes(times);
      double[] data = parameterVector.getData();
      double[][] splineCoefficients = new double[numberOfParametersPerSpline][numberOfVariables];
      for (int i = 0; i < numberOfSplines; i++){
         for (int j = 0; j < numberOfParametersPerSpline; j++){
            splineCoefficients[j] = Arrays.copyOfRange(data, (numberOfParametersPerSpline * i + j) * numberOfVariables, (numberOfParametersPerSpline * i + j + 1) * numberOfVariables);
         }
         toReturn.setSplineCoefficients(i, splineCoefficients);
      }
      return toReturn;
   }

   public int getNumberOfSplines(){
      return  numberOfSplines;
   }
   /**
    *
    * @param times the array of the timePoints, set time to NaN if you wish it to be set automatically (not valid behavior for start and end times
    */
   public void setTimes(double times[]){
      int lastTimeSet = -1;
      for (int i = 0; i < times.length; i++){
         if (i == 0 || i == times.length -1){
            if (Double.isNaN(times[i])) throw new RuntimeException(this.getClass().getSimpleName() + ": Invalid times");
         }

         if (!Double.isNaN(times[i])){
            if (lastTimeSet >= 0)
            {
               if (times[i] <= times[lastTimeSet])
                  throw new RuntimeException(this.getClass().getSimpleName() + ": Invalid times"); //time ordering
               this.times[i] = times[i];

               int j = i - 1;
               while (j > 0 && Double.isNaN(times[j]))
               {
                  this.times[j] = times[lastTimeSet] + (times[i] - times[lastTimeSet]) * ((double)(j - lastTimeSet)) /((double) (i - lastTimeSet));
                  j--;
               }
            }else{
               this.times[i] = times[i];
               lastTimeSet = i;
            }
         }
      }
      timesSet = true;
   }

   private int getSplineIndex(double time){
      if (time < times[0] || time > times[times.length -1]) throw new RuntimeException(this.getClass().getSimpleName() + ": Time out of Range");
      int index = 0;
      for (int i = 0; i < times.length; i++){
         if (time > times[i]){
            index = i;
         }
      }
      return index;
   }

   public void setPositionConstraint(double time, double[] constraintValues){
      setConstraint(0, time, constraintValues);
   }

   public void setVelocityConstraint(double time, double[] constraintValues){
      setConstraint(1, time, constraintValues);
   }

   public void setAccelerationConstraint(double time, double[] constraintValues){
      setConstraint(2, time, constraintValues);
   }

   public void setConstraint(int derivativeNumber, double time, double[] constraintValues){
      if (constraintValues.length != numberOfVariables) throw new RuntimeException(this.getClass().getSimpleName() + ": Invalid Constraint Terms");
      if (numberOfConstraintsSet == numberOfConstraints) throw new RuntimeException(this.getClass().getSimpleName() + ": Too Many Constraints");
      if (!timesSet) throw new RuntimeException(this.getClass().getSimpleName() + ": Set Times before Constraints");

      double[] values = getOrderValues(derivativeNumber, time);

      int splineIndex = getSplineIndex(time);
      int startRow = currentRow * numberOfVariables;
      int startColumn = splineIndex * numberOfParametersPerSpline * numberOfVariables;
      //set terms in the constraint matrix
      setPatternBlock(startRow, startColumn, constraintMatrix, values, numberOfVariables);
      //set terms in the constraintVector
      setColumnBlock(startRow, 0, constraintVector, constraintValues);

      numberOfConstraintsSet++;
      currentRow++;
   }

   private double[] getOrderValues(int derivativeNumber, double time){
      int[] coefficients = new int[numberOfParametersPerSpline];
      int[] order = new int[numberOfParametersPerSpline];
      for (int i = 0; i < numberOfParametersPerSpline; i++){
         coefficients[i] = 1;
         order[i] = Math.max(0, orderOfSplines - i);
      }
      for (int i = 0; i < derivativeNumber; i++){
         for (int j = 0; j < numberOfParametersPerSpline; j++){
            coefficients[j] *= order[j];
            order[j] = Math.max(0, order[j] -1);
         }
      }

      double[] values = new double[numberOfParametersPerSpline];
      for (int i = 0; i < numberOfParametersPerSpline; i++){
         values[i] = coefficients[i] * Math.pow(time, order[i]);
      }
      return values;
   }

   private void setContinuityConstraint(int secondSplineIndex, int derivativeNumber, double timeAtConstraint){
      double[] values = getOrderValues(derivativeNumber, timeAtConstraint);
      int startRow = currentRow * numberOfVariables;
      int columnIndex1 = (secondSplineIndex - 1) * numberOfParametersPerSpline * numberOfVariables;
      int columnIndex2 = (secondSplineIndex) * numberOfParametersPerSpline * numberOfVariables;
      setPatternBlock(startRow, columnIndex1, constraintMatrix, values, numberOfVariables, false);
      setPatternBlock(startRow, columnIndex2, constraintMatrix, values, numberOfVariables, true);
      currentRow++;
   }

   private void setPatternBlock(int rowStart, int columnStart, DenseMatrix64F matrix64F, double[] pattern, int numberOfVariables){
      setPatternBlock(rowStart, columnStart, matrix64F, pattern, numberOfVariables, false);
   }

   private void setPatternBlock(int rowStart, int columnStart, DenseMatrix64F matrix64F, double[] pattern, int numberOfVariables, boolean inverted){
      double sign = inverted ? -1.0: 1.0;
      for (int i =  0; i < numberOfVariables; i++){
         for (int j = 0; j < pattern.length; j++){
            matrix64F.set(rowStart + i, columnStart + j*numberOfVariables+i, sign * pattern[j]);
         }
      }
   }

   private void setColumnBlock(int rowStart, int columnStart, DenseMatrix64F matrix64F, double[] block){
      for (int i = 0; i < block.length; i++){
         matrix64F.set(rowStart + i, columnStart, block[i]);
      }
   }
}
