package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DMatrixRMaj;

public class QPInputTypeC
{
   private int numberOfVariables;

   public final DMatrixRMaj taskWeightMatrix = new DMatrixRMaj(0, 0);
   public final DMatrixRMaj directCostHessian = new DMatrixRMaj(0, 0);
   public final DMatrixRMaj directCostGradient = new DMatrixRMaj(0, 0);

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
    * <li>u is the general objective
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
   public QPInputTypeC(int numberOfVariables)
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


   public void setTaskWeightMatrix(DMatrixRMaj taskWeightMatrix)
   {
      this.taskWeightMatrix.set(taskWeightMatrix);
   }

   public DMatrixRMaj getTaskWeightMatrix()
   {
      return taskWeightMatrix;
   }

   public void setDirectCostHessian(DMatrixRMaj directCostHessian)
   {
      this.directCostHessian.set(directCostHessian);
   }

   public DMatrixRMaj getDirectCostHessian()
   {
      return directCostHessian;
   }

   public void setDirectCostGradient(DMatrixRMaj directCostGradient)
   {
      this.directCostGradient.set(directCostGradient);
   }

   public DMatrixRMaj getDirectCostGradient()
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
