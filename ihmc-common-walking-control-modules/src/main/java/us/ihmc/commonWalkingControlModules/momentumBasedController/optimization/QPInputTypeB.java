package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DMatrixRMaj;

public class QPInputTypeB
{
   private static final int initialTaskSize = 6;

   private final int numberOfVariables;

   public final DMatrixRMaj taskJacobian = new DMatrixRMaj(0, 0);
   public final DMatrixRMaj taskConvectiveTerm = new DMatrixRMaj(0, 0);
   public final DMatrixRMaj directCostHessian = new DMatrixRMaj(0, 0);
   public final DMatrixRMaj directCostGradient = new DMatrixRMaj(0, 0);

   private double taskWeight;

   /**
    * <p>
    * Direct input into the QP solver. This is only an objective cost function Must be in the form
    * </p>
    * <p>
    * w (0.5 * u^T * H * u + g * u)
    * </p>
    * <p>
    *    u = A * x + b
    * </p>
    * where:
    * <ul>
    * <li>A is {@link #taskJacobian}</li>
    * <li>b is {@link #taskConvectiveTerm}</li>
    * <li>u is the general objective</li>
    * <li>H is {@link #directCostHessian}</li>
    * <li>g is {@link #directCostGradient}</li>
    * <li>w is the overall weight scaling of htis task</li>
    * <li>x is the vector of the problem variables, for instance joint accelerations.</li>
    *
    * This cost function is then expanded out as
    * <pre>
    * f(x) = w * (0.5 * x<sup>T</sup> * A<sup>T</sup> * Q * A * x - b<sup>T </sup> Q * A * x )
    * </pre>
    */
   public QPInputTypeB(int numberOfVariables)
   {
      this.numberOfVariables = numberOfVariables;
      reshape(initialTaskSize);
   }


   public void reshape(int taskSize)
   {
      taskJacobian.reshape(taskSize, numberOfVariables);
      taskConvectiveTerm.reshape(taskSize, 1);
      directCostHessian.reshape(taskSize, taskSize);
      directCostGradient.reshape(taskSize, 1);
   }

   public void setTaskJacobian(DMatrixRMaj taskJacobian)
   {
      this.taskJacobian.set(taskJacobian);
   }

   public DMatrixRMaj getTaskJacobian()
   {
      return taskJacobian;
   }

   public void setTaskConvectiveTerm(DMatrixRMaj taskConvectiveTerm)
   {
      this.taskConvectiveTerm.set(taskConvectiveTerm);
   }

   public DMatrixRMaj getTaskConvectiveTerm()
   {
      return taskConvectiveTerm;
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

   public void setWeight(double weight)
   {
      this.taskWeight = weight;
   }

   public double getWeight()
   {
      return taskWeight;
   }

   public double getTaskWeight()
   {
      return taskWeight;
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName();
      ret += "Jacobian:\n" + taskJacobian;
      ret += "Convective Term:\n" + taskConvectiveTerm;
      ret += "Direct Hessian: \n" + directCostHessian;
      ret += "Direct Gradient: \n" + directCostGradient;
      ret += "Weight: " + taskWeight;
      return ret;
   }
}
