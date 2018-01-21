package us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.trajectoryOptimization.DefaultDiscreteState;
import us.ihmc.trajectoryOptimization.DiscreteHybridDynamics;

/** This class defines the Linear Inverted Pendulum Dynamics.
 * The state variables are X = [x, y, z, xdot, ydot, zdot], the center of mass
 * position and velocity.
 * The control variables are U = [px, py, Fz], the CoP position and vertical force.
 */
public class LIPMDynamics implements DiscreteHybridDynamics<DefaultDiscreteState>
{
   private static final boolean incorporateAccelerationIntoPosition = true;
   static final int stateVectorSize = 6;
   static final int controlVectorSize = 3;
   static final int constantVectorSize = 0;

   private double deltaT;
   private double deltaT2;
   private final double pendulumMass;
   private final double gravityZ;

   public LIPMDynamics(double deltaT, double pendulumMass, double gravityZ)
   {
      this.deltaT = deltaT;
      this.deltaT2 = deltaT * deltaT;
      this.pendulumMass = pendulumMass;
      this.gravityZ = gravityZ;
   }

   @Override
   public void setTimeStepSize(double deltaT)
   {
      this.deltaT = deltaT;
      deltaT2 = deltaT * deltaT;
   }

   public int getStateVectorSize()
   {
      return stateVectorSize;
   }

   public int getControlVectorSize()
   {
      return controlVectorSize;
   }

   public int getConstantVectorSize()
   {
      return constantVectorSize;
   }

   /** Returns the current dynamics, of the form
    * X_k+1 = f(X_k, U_k)
    * @param state state of the current dynamics. For LIPM, this doesn't change
    * @param currentState X_k in the above equation
    * @param currentControl U_k in the above equation
    * @param matrixToPack f(X_k, U_k) in the above equation
    */
   @Override
   public void getNextState(DefaultDiscreteState state, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F constants,
                            DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.numRows != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (matrixToPack.numCols != 1)
         throw new RuntimeException("The state matrix size is wrong.");

      double x_k = currentState.get(0);
      double y_k = currentState.get(1);
      double z_k = currentState.get(2);
      double xdot_k = currentState.get(3);
      double ydot_k = currentState.get(4);
      double zdot_k = currentState.get(5);

      double px_k = currentControl.get(0);
      double py_k = currentControl.get(1);
      double fz_k = currentControl.get(2);

      double x_k1 = x_k + deltaT * xdot_k;
      double y_k1 = y_k + deltaT * ydot_k;
      double z_k1 = z_k + deltaT * zdot_k;
      if (incorporateAccelerationIntoPosition)
      {
         x_k1 += 0.5 * deltaT2 * (x_k - px_k) * fz_k / (pendulumMass * z_k);
         y_k1 += 0.5 * deltaT2 * (y_k - py_k) * fz_k / (pendulumMass * z_k);
         z_k1 += 0.5 * deltaT2 * (fz_k / pendulumMass - gravityZ);
      }

      double xdot_k1 = xdot_k + deltaT * (x_k - px_k) * fz_k / (pendulumMass * z_k);
      double ydot_k1 = ydot_k + deltaT * (y_k - py_k) * fz_k / (pendulumMass * z_k);
      double zdot_k1 = zdot_k + deltaT * (fz_k / pendulumMass - gravityZ);

      matrixToPack.set(0, 0, x_k1);
      matrixToPack.set(1, 0, y_k1);
      matrixToPack.set(2, 0, z_k1);
      matrixToPack.set(3, 0, xdot_k1);
      matrixToPack.set(4, 0, ydot_k1);
      matrixToPack.set(5, 0, zdot_k1);
   }

   @Override
   public void getDynamicsStateGradient(DefaultDiscreteState state, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F constants,
                                        DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.numRows != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (matrixToPack.numCols != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");

      double x_k = currentState.get(0);
      double y_k = currentState.get(1);
      double z_k = currentState.get(2);

      double px_k = currentControl.get(0);
      double py_k = currentControl.get(1);
      double fz_k = currentControl.get(2);

      matrixToPack.zero();

      double value1 = deltaT2 / (2.0 * pendulumMass * z_k);
      double value2 = deltaT / (pendulumMass * z_k);

      if (incorporateAccelerationIntoPosition)
      {
         matrixToPack.set(0, 0, 1 + fz_k * value1);
         matrixToPack.set(0, 2, (px_k - x_k) * fz_k / z_k * value1);
         matrixToPack.set(0, 3, deltaT);

         matrixToPack.set(1, 1, matrixToPack.get(0, 0));
         matrixToPack.set(1, 2, (py_k - y_k) * fz_k / z_k * value1);
         matrixToPack.set(1, 4, deltaT);

         matrixToPack.set(2, 2, 1.0);
         matrixToPack.set(2, 5, deltaT);

         matrixToPack.set(3, 0, fz_k * value2);
         matrixToPack.set(3, 2, (px_k - x_k) * fz_k / z_k * value2);
         matrixToPack.set(3, 3, 1.0);

         matrixToPack.set(4, 1, matrixToPack.get(3, 0));
         matrixToPack.set(4, 2, (py_k - y_k) * fz_k / z_k * value2);
         matrixToPack.set(4, 4, 1.0);

         matrixToPack.set(5, 5, 1.0);
      }
      else
      {
         CommonOps.setIdentity(matrixToPack);

         matrixToPack.set(0, 3, deltaT);
         matrixToPack.set(1, 4, deltaT);
         matrixToPack.set(2, 5, deltaT);

         matrixToPack.set(3, 0, fz_k * deltaT / (pendulumMass * z_k));
         matrixToPack.set(3, 2, fz_k * deltaT * (px_k - x_k) / (pendulumMass * z_k * z_k));

         matrixToPack.set(4, 1, fz_k * deltaT / (pendulumMass * z_k));
         matrixToPack.set(4, 2, fz_k * deltaT * (py_k - y_k) / (pendulumMass * z_k * z_k));
      }
   }

   @Override
   public void getDynamicsControlGradient(DefaultDiscreteState state, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F constants,
                                          DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.numRows != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (matrixToPack.numCols != controlVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");

      double x_k = currentState.get(0);
      double y_k = currentState.get(1);
      double z_k = currentState.get(2);

      double px_k = currentControl.get(0);
      double py_k = currentControl.get(1);
      double fz_k = currentControl.get(2);

      matrixToPack.zero();

      double value = deltaT / (pendulumMass * z_k);
      double value2 = deltaT2 / (2 * pendulumMass * z_k);

      if (incorporateAccelerationIntoPosition)
      {
         matrixToPack.set(0, 0, value2 * -fz_k);
         matrixToPack.set(0, 2, value2 * (x_k - px_k));

         matrixToPack.set(1, 1, value2 * -fz_k);
         matrixToPack.set(1, 2, value2 * (y_k - py_k));

         matrixToPack.set(2, 2, value2 * z_k);

         matrixToPack.set(3, 0, value * -fz_k);
         matrixToPack.set(3, 2, value * (x_k - px_k));

         matrixToPack.set(4, 1, value * -fz_k);
         matrixToPack.set(4, 2, value * (y_k - py_k));

         matrixToPack.set(5, 2, value * z_k);
      }
      else
      {
         matrixToPack.set(3, 0, -fz_k * deltaT / (pendulumMass * z_k));
         matrixToPack.set(3, 2, -(px_k - x_k) * deltaT / (pendulumMass * z_k));

         matrixToPack.set(4, 1, -fz_k * deltaT / (pendulumMass * z_k));
         matrixToPack.set(4, 2, -(py_k - y_k) * deltaT / (pendulumMass * z_k));

         matrixToPack.set(5, 2, deltaT / pendulumMass);
      }
   }

   /**
    * {@param stateVariable} represents the first partial derivative variable of the dynamics. The gradient is then taken w.r.t to this value.
    */
   @Override
   public void getDynamicsStateHessian(DefaultDiscreteState state, int stateVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl,
                                       DenseMatrix64F constants, DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.numRows != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (matrixToPack.numCols != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (stateVariable >= stateVectorSize)
         throw new RuntimeException("Too big a state variable.");

      double x_k = currentState.get(0);
      double y_k = currentState.get(1);
      double z_k = currentState.get(2);

      double px_k = currentControl.get(0);
      double py_k = currentControl.get(1);
      double fz_k = currentControl.get(2);

      matrixToPack.zero();

      double value = -deltaT * fz_k / (pendulumMass * z_k * z_k);
      double value2 = -deltaT2 * fz_k / (2 * pendulumMass * z_k * z_k);

      if (incorporateAccelerationIntoPosition)
      {
         switch (stateVariable)
         {
         case 0:
            matrixToPack.set(0, 2, value2);
            matrixToPack.set(3, 2, value);
            break;
         case 1:
            matrixToPack.set(1, 2, value2);
            matrixToPack.set(4, 2, value);
            break;
         case 2:
            matrixToPack.set(0, 0, value2);
            matrixToPack.set(0, 2, 2.0 * value2 * (px_k - x_k) / z_k);

            matrixToPack.set(1, 1, value2);
            matrixToPack.set(1, 2, 2.0 * value2 * (py_k - y_k) / z_k);

            matrixToPack.set(3, 0, value);
            matrixToPack.set(3, 2, 2.0 * value * (px_k - x_k) / z_k);

            matrixToPack.set(4, 1, value);
            matrixToPack.set(4, 2, 2.0 * value * (py_k - y_k) / z_k);
            break;
         }
      }
      else
      {
         switch (stateVariable)
         {
         case 0:
            matrixToPack.set(3, 2, value);
            break;
         case 1:
            matrixToPack.set(4, 2, value);
            break;
         case 2:
            matrixToPack.set(3, 0, value);
            matrixToPack.set(3, 2, 2.0 * value * (px_k - x_k) / z_k);

            matrixToPack.set(4, 1, value);
            matrixToPack.set(4, 2, 2.0 * value * (py_k - y_k) / z_k);
            break;
         }
      }
   }

   @Override
   public void getDynamicsControlHessian(DefaultDiscreteState state, int controlVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl,
                                         DenseMatrix64F constants, DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.numRows != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (matrixToPack.numCols != controlVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (controlVariable >= controlVectorSize)
         throw new RuntimeException("Too big a state variable.");

      double z_k = currentState.get(2);

      matrixToPack.zero();

      double value = -deltaT / (pendulumMass * z_k);
      double value2 = -deltaT2 / (2 * pendulumMass * z_k);

      if (incorporateAccelerationIntoPosition)
      {
         switch (controlVariable)
         {
         case 0:
            matrixToPack.set(0, 2, value2);
            matrixToPack.set(3, 2, value);
            break;
         case 1:
            matrixToPack.set(1, 2, value2);
            matrixToPack.set(4, 2, value);
            break;
         case 2:
            matrixToPack.set(0, 0, value2);
            matrixToPack.set(1, 1, value2);
            matrixToPack.set(3, 0, value);
            matrixToPack.set(4, 1, value);
            break;
         }
      }
      else
      {
         switch (controlVariable)
         {
         case 0:
            matrixToPack.set(3, 2, value);
            break;
         case 1:
            matrixToPack.set(4, 2, value);
            break;
         case 2:
            matrixToPack.set(3, 0, value);
            matrixToPack.set(4, 1, value);
            break;
         }
      }
   }

   /** f_ux */
   @Override
   public void getDynamicsStateGradientOfControlGradient(DefaultDiscreteState state, int stateVariable, DenseMatrix64F currentState,
                                                         DenseMatrix64F currentControl, DenseMatrix64F constants, DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.numRows != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (matrixToPack.numCols != controlVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (stateVariable >= stateVectorSize)
         throw new RuntimeException("Too big a state variable.");

      double x_k = currentState.get(0);
      double y_k = currentState.get(1);
      double z_k = currentState.get(2);

      double px_k = currentControl.get(0);
      double py_k = currentControl.get(1);
      double fz_k = currentControl.get(2);

      matrixToPack.zero();

      double value = deltaT / (pendulumMass * z_k * z_k);
      double value2 = deltaT2 / (2 * pendulumMass * z_k * z_k);

      if (incorporateAccelerationIntoPosition)
      {
         switch (stateVariable)
         {
         case 0:
            matrixToPack.set(0, 2, value2 * z_k);
            matrixToPack.set(3, 2, value * z_k);
            break;
         case 1:
            matrixToPack.set(1, 2, value2 * z_k);
            matrixToPack.set(4, 2, value * z_k);
            break;
         case 2:
            matrixToPack.set(0, 0, value2 * fz_k);
            matrixToPack.set(0, 2, value2 * (px_k - x_k));

            matrixToPack.set(1, 1, value2 * fz_k);
            matrixToPack.set(1, 2, value2 * (py_k - y_k));

            matrixToPack.set(3, 0, value * fz_k);
            matrixToPack.set(3, 2, value * (px_k - x_k));

            matrixToPack.set(4, 1, value * fz_k);
            matrixToPack.set(4, 2, value * (py_k - y_k));
            break;
         }
      }
      else
      {
         switch (stateVariable)
         {
         case 0:
            matrixToPack.set(3, 2, value * z_k);
            break;
         case 1:
            matrixToPack.set(4, 2, value * z_k);
            break;
         case 2:
            matrixToPack.set(3, 0, value * fz_k);
            matrixToPack.set(3, 2, value * (px_k - x_k));

            matrixToPack.set(4, 1, value * fz_k);
            matrixToPack.set(4, 2, value * (py_k - y_k));
            break;
         }
      }
   }

   /** f_xu */
   @Override
   public void getDynamicsControlGradientOfStateGradient(DefaultDiscreteState state, int controlVariable, DenseMatrix64F currentState,
                                                         DenseMatrix64F currentControl, DenseMatrix64F constants, DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.numRows != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (matrixToPack.numCols != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (controlVariable >= controlVectorSize)
         throw new RuntimeException("Too big a control variable.");

      double x_k = currentState.get(0);
      double y_k = currentState.get(1);
      double z_k = currentState.get(2);

      double px_k = currentControl.get(0);
      double py_k = currentControl.get(1);
      double fz_k = currentControl.get(2);

      matrixToPack.zero();

      double value = deltaT / (pendulumMass * z_k * z_k );
      double value2 = deltaT2 / (2 * pendulumMass * z_k * z_k );

      if (incorporateAccelerationIntoPosition)
      {
         switch (controlVariable)
         {
         case 0:
            matrixToPack.set(0, 2, fz_k * value2);
            matrixToPack.set(3, 2, fz_k * value);
            break;
         case 1:
            matrixToPack.set(1, 2, fz_k * value2);
            matrixToPack.set(4, 2, fz_k * value);
            break;
         case 2:
            matrixToPack.set(0, 0, value2 * z_k);
            matrixToPack.set(0, 2, value2 * (px_k - x_k));

            matrixToPack.set(1, 1, value2 * z_k);
            matrixToPack.set(1, 2, value2 * (py_k - y_k));

            matrixToPack.set(3, 0, value * z_k);
            matrixToPack.set(3, 2, value * (px_k - x_k));

            matrixToPack.set(4, 1, value * z_k);
            matrixToPack.set(4, 2, value * (py_k - y_k));
         }
      }
      else
      {
         switch (controlVariable)
         {
         case 0:
            matrixToPack.set(3, 2, fz_k * value);
            break;
         case 1:
            matrixToPack.set(4, 2, fz_k * value);
            break;
         case 2:
            matrixToPack.set(3, 0, value * z_k);
            matrixToPack.set(3, 2, value * (px_k - x_k));

            matrixToPack.set(4, 1, value * z_k);
            matrixToPack.set(4, 2, value * (py_k - y_k));
         }
      }
   }

   @Override
   public void getContinuousAMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(0, 2, 1.0);
      matrixToPack.set(1, 3, 1.0);
      matrixToPack.set(2, 0, gravityZ / 1.0);
      matrixToPack.set(3, 1, gravityZ / 1.0);
   }

   @Override
   public void getContinuousBMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(2, 0, -gravityZ / 1.0);
      matrixToPack.set(3, 1, -gravityZ / 1.0);
   }
}
