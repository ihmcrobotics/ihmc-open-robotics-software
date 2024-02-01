package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.matrixlib.NativeMatrix;

import java.lang.annotation.Native;

public class NativeQPInputTypeA
{
   private static final int initialTaskSize = 6;

   private int numberOfVariables;

   public final NativeMatrix taskJacobian = new NativeMatrix(0, 0);
   public final NativeMatrix taskObjective = new NativeMatrix(0, 0);
   public final NativeMatrix taskWeightMatrix = new NativeMatrix(0, 0);

   private boolean useWeightScalar = false;
   private double taskWeightScalar;

   private ConstraintType constraintType = ConstraintType.OBJECTIVE;

   /**
    * <p>
    * Input into the QP solver.
    *
    * If formulated as an objective, it tries to minimize the squared difference function in the form
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
    *
    * <p>
    * If specified as an objective using {@code constraintType}, the cost function is then expanded out as
    * </p>
    * <p>
    * f(x) = 0.5 * x<sup>T</sup> * A<sup>T</sup> * Q * A * x - b<sup>T </sup> Q * A * x
    * </p>
    *
    * <p>
    * If formulated as a equality, it is in the form
    * </p>
    * <p>
    *    A * x = b
    * </p>
    *
    * <p>
    *    If formulated as a greater than/equal inequality, it is in the form
    * </p>
    * <p>
    *    A * x >= b
    * </p>
    * <p>
    * If formulated as a less than/equal inequality, it is in the form
    * </p>
    * <p>
    *    A * x <= b
    * </p>
    */
   public NativeQPInputTypeA(int numberOfVariables)
   {
      this.numberOfVariables = numberOfVariables;
      reshape(initialTaskSize);
   }

   public void set(QPInputTypeA inputTypeA)
   {
      setTaskJacobian(inputTypeA.getTaskJacobian());
      setTaskObjective(inputTypeA.getTaskObjective());
      setUseWeightScalar(inputTypeA.useWeightScalar());
      setNumberOfVariables(inputTypeA.taskJacobian.getNumCols());
      setConstraintType(inputTypeA.getConstraintType());
      if (useWeightScalar)
         setWeight(inputTypeA.getWeightScalar());
      else
         setTaskWeightMatrix(inputTypeA.getTaskWeightMatrix());
   }

   public void setNumberOfVariables(int numberOfVariables)
   {
      this.numberOfVariables = numberOfVariables;
   }

   public int getNumberOfVariables()
   {
      return numberOfVariables;
   }

   public void reshape(int taskSize)
   {
      taskJacobian.reshape(taskSize, numberOfVariables);
      taskObjective.reshape(taskSize, 1);
      taskWeightMatrix.reshape(taskSize, taskSize);
      taskJacobian.zero();
      taskObjective.zero();
      taskWeightMatrix.zero();
   }

   public void setTaskJacobian(DMatrix taskJacobian)
   {
      this.taskJacobian.set(taskJacobian);
   }

   public NativeMatrix getTaskJacobian()
   {
      return taskJacobian;
   }

   public void setTaskObjective(DMatrix taskObjective)
   {
      this.taskObjective.set(taskObjective);
   }

   public NativeMatrix getTaskObjective()
   {
      return taskObjective;
   }

   public void setTaskWeightMatrix(DMatrix taskWeightMatrix)
   {
      this.taskWeightMatrix.set(taskWeightMatrix);
   }

   public NativeMatrix getTaskWeightMatrix()
   {
      return taskWeightMatrix;
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

   public double getTaskWeightScalar()
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
