package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;

public class MotionQPInput
{
   public final DenseMatrix64F taskJacobian;
   public final DenseMatrix64F taskObjective;
   public final DenseMatrix64F taskWeightMatrix;
   private double taskWeightScalar;
   private boolean useWeightScalar = false;
   private final int numberOfDoFs;
   private ConstraintType constraintType = ConstraintType.OBJECTIVE;

   /**
    * <p>
    * Motion objective input into the QP solver. Must be in the form
    * </p>
    * <p>
    * A * x - b
    * </p>
    * where:
    * <ul>
    * <li>A is {@code taskJacobian}
    * <li>b is {@code taskObjective}
    * <li>x is the vector of the problem variables, for instance joint accelerations.
    * <p>
    * where the overall desire is minimize the objective.
    * </p>
    */
   public MotionQPInput(int numberOfDoFs)
   {
      this.numberOfDoFs = numberOfDoFs;
      taskJacobian = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
      taskObjective = new DenseMatrix64F(numberOfDoFs, 1);
      taskWeightMatrix = new DenseMatrix64F(numberOfDoFs, numberOfDoFs);
   }

   public void reshape(int taskSize)
   {
      taskJacobian.reshape(taskSize, numberOfDoFs);
      taskObjective.reshape(taskSize, 1);
      taskWeightMatrix.reshape(taskSize, taskSize);
   }

   public void setTaskJacobian(DenseMatrix64F taskJacobian)
   {
      this.taskJacobian.set(taskJacobian);
   }

   public void setTaskObjective(DenseMatrix64F taskObjective)
   {
      this.taskObjective.set(taskObjective);
   }

   public void setTaskWeightMatrix(DenseMatrix64F taskWeightMatrix)
   {
      this.taskWeightMatrix.set(taskWeightMatrix);
   }

   public void setUseWeightScalar(boolean useWeightScalar)
   {
      this.useWeightScalar = useWeightScalar;
   }

   public void setWeight(double weight)
   {
      this.taskWeightScalar = weight;
   }

   public double getWeightScalar()
   {
      return taskWeightScalar;
   }

   public boolean useWeightScalar()
   {
      return useWeightScalar;
   }

   public void setConstraintType(ConstraintType constraintType)
   {
      this.constraintType = constraintType;
   }

   public ConstraintType getConstraintType()
   {
      return constraintType;
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName();
      ret += "Jacobian:\n" + taskJacobian;
      ret += "Objective:\n" + taskObjective;
      if (constraintType != ConstraintType.OBJECTIVE)
         ret += constraintType.toString();
      else if (useWeightScalar)
         ret += "Weight: " + taskWeightScalar;
      else
         ret += "Weight:\n" + taskWeightMatrix;
      return ret;
   }
}
