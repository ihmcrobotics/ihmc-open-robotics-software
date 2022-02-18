package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.matrixlib.NativeMatrix;

public class NativeQPInputTypeC
{
   private int numberOfVariables;

   public final NativeMatrix taskWeightMatrix = new NativeMatrix(0, 0);
   public final NativeMatrix directCostHessian = new NativeMatrix(0, 0);
   public final NativeMatrix directCostGradient = new NativeMatrix(0, 0);

   private boolean useWeightScalar = false;
   private double taskWeightScalar;

   /**
    * <p>
    * Direct input into the QP solver. This is only an objective cost function Must be in the form
    * </p>
    * <p>
    * g * Q * x + 0.5 * x^T * (H + Q) * x
    * </p>
    * where:
    * <ul>
    * <li>Q is {@link #taskWeightMatrix}
    * <li>H is {@link #directCostHessian}
    * <li>g is {@link #directCostGradient}
    * <li>x is the vector of the problem variables, for instance joint accelerations.
    *
    * This cost function is then expanded out as
    * <pre>
    * f(x) = 0.5 * x<sup>T</sup> * Q * x - b<sup>T </sup> Q * x
    * </pre>
    */
   public NativeQPInputTypeC(int numberOfVariables)
   {
      this.numberOfVariables = numberOfVariables;
   }

   public void setNumberOfVariables(int numberOfVariables)
   {
      this.numberOfVariables = numberOfVariables;
   }

   public void reshape()
   {
      taskWeightMatrix.reshape(numberOfVariables, numberOfVariables);
      directCostHessian.reshape(numberOfVariables, numberOfVariables);
      directCostGradient.reshape(numberOfVariables, 1);
   }


   public void setTaskWeightMatrix(DMatrix taskWeightMatrix)
   {
      this.taskWeightMatrix.set(taskWeightMatrix);
   }

   public NativeMatrix getTaskWeightMatrix()
   {
      return taskWeightMatrix;
   }

   public void setDirectCostHessian(DMatrix directCostHessian)
   {
      this.directCostHessian.set(directCostHessian);
   }

   public NativeMatrix getDirectCostHessian()
   {
      return directCostHessian;
   }

   public void setDirectCostGradient(DMatrix directCostGradient)
   {
      this.directCostGradient.set(directCostGradient);
   }

   public NativeMatrix getDirectCostGradient()
   {
      return directCostGradient;
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


   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName();
      ret += "Direct Hessian: \n" + directCostHessian;
      ret += "Direct Gradient: \n" + directCostGradient;
      if (useWeightScalar)
         ret += "Weight: " + taskWeightScalar;
      else
         ret += "Weight:\n" + taskWeightMatrix;
      return ret;
   }
}
