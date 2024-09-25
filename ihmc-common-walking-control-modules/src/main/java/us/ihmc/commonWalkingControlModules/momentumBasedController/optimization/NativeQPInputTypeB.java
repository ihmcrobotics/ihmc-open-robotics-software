package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.matrixlib.NativeMatrix;

public class NativeQPInputTypeB
{
   private static final int initialTaskSize = 6;

   private final int numberOfVariables;

   public final NativeMatrix taskJacobian = new NativeMatrix(0, 0);
   public final NativeMatrix taskConvectiveTerm = new NativeMatrix(0, 0);
   public final NativeMatrix directCostHessian = new NativeMatrix(0, 0);
   public final NativeMatrix directCostGradient = new NativeMatrix(0, 0);

   private double taskWeight;

   /**
    * <p>
    * Direct input into the QP solver. This is only an objective cost function Must be in the form
    * </p>
    * <p>
    * w (0.5 * u^T * H * u + g * u)
    * </p>
    * <p>
    *    u = J * x + b
    * </p>
    * where:
    * <ul>
    * <li>J is {@link #taskJacobian}</li>
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
   public NativeQPInputTypeB(int numberOfVariables)
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

   public NativeMatrix getTaskJacobian()
   {
      return taskJacobian;
   }

   public void setTaskConvectiveTerm(DMatrixRMaj taskConvectiveTerm)
   {
      this.taskConvectiveTerm.set(taskConvectiveTerm);
   }

   public NativeMatrix getTaskConvectiveTerm()
   {
      return taskConvectiveTerm;
   }

   public void setDirectCostHessian(DMatrixRMaj directCostHessian)
   {
      this.directCostHessian.set(directCostHessian);
   }

   public NativeMatrix getDirectCostHessian()
   {
      return directCostHessian;
   }

   public void setDirectCostGradient(DMatrixRMaj directCostGradient)
   {
      this.directCostGradient.set(directCostGradient);
   }

   public NativeMatrix getDirectCostGradient()
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
