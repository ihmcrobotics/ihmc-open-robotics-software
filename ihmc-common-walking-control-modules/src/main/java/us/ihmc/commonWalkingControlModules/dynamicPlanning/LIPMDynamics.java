package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;

/** This class defines the Linear Inverted Pendulum Dynamics.
 * The state variables are X = [x, y, z, xdot, ydot, zdot], the center of mass
 * position and velocity.
 * The control variables are U = [px, py, Fz], the CoP position and vertical force.
 */
public class LIPMDynamics implements DiscreteHybridDynamics<LIPMState>
{
   static final int stateVectorSize = 6;
   static final int controlVectorSize = 3;

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

   /** Returns the current dynamics, of the form
    * X_k+1 = f(X_k, U_k)
    * @param hybridState state of the current dynamics. For LIPM, this doesn't change
    * @param currentState X_k in the above equation
    * @param currentControl U_k in the above equation
    * @param matrixToPack f(X_k, U_k) in the above equation
    */
   public void getDynamics(LIPMState hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F matrixToPack)
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

      double x_k1 = x_k + deltaT * xdot_k + 0.5 * deltaT2 * (x_k - px_k) * fz_k / (pendulumMass * z_k);
      double y_k1 = y_k + deltaT * ydot_k + 0.5 * deltaT2 * (y_k - py_k) * fz_k / (pendulumMass * z_k);
      double z_k1 = z_k + deltaT * zdot_k + 0.5 * deltaT2 * (fz_k / pendulumMass - gravityZ);

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

   public void getDynamicsStateGradient(LIPMState hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F matrixToPack)
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

      matrixToPack.set(0, 0, 1 + deltaT2 * fz_k / (2.0 * pendulumMass * z_k));
      matrixToPack.set(0, 2, deltaT2 * (px_k - x_k) * fz_k / (2.0 * pendulumMass * z_k * z_k));
      matrixToPack.set(0, 3, deltaT);

      matrixToPack.set(1, 1, matrixToPack.get(0, 0));
      matrixToPack.set(1, 2, deltaT2 * (py_k - y_k) * fz_k / (2.0 * pendulumMass * z_k * z_k));
      matrixToPack.set(1, 4, deltaT);

      matrixToPack.set(2, 2, 1.0);
      matrixToPack.set(2, 5, deltaT);

      matrixToPack.set(3, 0, deltaT * fz_k / (pendulumMass * z_k));
      matrixToPack.set(3, 2, deltaT * (px_k - x_k) * fz_k / (pendulumMass * z_k * z_k));
      matrixToPack.set(3, 3, 1.0);

      matrixToPack.set(4, 1, deltaT * fz_k / (pendulumMass * z_k));
      matrixToPack.set(4, 2, deltaT * (py_k - y_k) * fz_k / (pendulumMass * z_k * z_k));
      matrixToPack.set(4, 4, 1.0);

      matrixToPack.set(5, 5, 1.0);
   }

   public void getDynamicsControlGradient(LIPMState hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.numRows != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (matrixToPack.numCols != controlVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");

      double z_k = currentState.get(2);

      double px_k = currentControl.get(0);
      double py_k = currentControl.get(1);
      double fz_k = currentControl.get(2);

      matrixToPack.zero();

      double value = -deltaT / (pendulumMass * z_k);
      double value2 = -deltaT2 / (2 * pendulumMass * z_k);

      matrixToPack.set(0, 0, value2 * fz_k);
      matrixToPack.set(0, 2, value2 * px_k);

      matrixToPack.set(1, 1, value2 * fz_k);
      matrixToPack.set(1, 2, value2 * py_k);

      matrixToPack.set(2, 2, 0.5 * deltaT2 / pendulumMass);

      matrixToPack.set(3, 0, value * fz_k);
      matrixToPack.set(3, 2, value * px_k);

      matrixToPack.set(4, 1, value * fz_k);
      matrixToPack.set(4, 2, value * py_k);

      matrixToPack.set(5, 2, deltaT / pendulumMass);
   }

   public void getDynamicsStateHessian(LIPMState hybridState, int stateVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F matrixToPack)
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
         matrixToPack.set(0, 2, value2 * (px_k - x_k) / z_k);

         matrixToPack.set(1, 1, value2);
         matrixToPack.set(1, 2, value2 * (py_k - y_k) / z_k);

         matrixToPack.set(3, 0, value);
         matrixToPack.set(3, 2, value * (px_k - x_k) / z_k);

         matrixToPack.set(4, 1, value);
         matrixToPack.set(4, 2, value * (py_k - y_k) / z_k);
         break;
      }
   }

   public void getDynamicsControlHessian(LIPMState hybridState, int controlVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F matrixToPack)
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

   /** f_ux */
   public void getDynamicsStateGradientOfControlGradient(LIPMState hybridState, int stateVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.numRows != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (matrixToPack.numCols != controlVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (stateVariable >= stateVectorSize)
         throw new RuntimeException("Too big a state variable.");

      double z_k = currentState.get(2);

      double px_k = currentControl.get(0);
      double py_k = currentControl.get(1);
      double fz_k = currentControl.get(2);

      matrixToPack.zero();

      double value = deltaT / (pendulumMass * z_k * z_k);
      double value2 = deltaT2 / (2 * pendulumMass * z_k * z_k);

      if (stateVariable == 2)
      {
         matrixToPack.set(0, 0, value2 * fz_k);
         matrixToPack.set(0, 2, value2 * px_k);

         matrixToPack.set(1, 1, value2 * fz_k);
         matrixToPack.set(1, 2, value2 * py_k);

         matrixToPack.set(3, 0, value * fz_k);
         matrixToPack.set(3, 2, value * px_k);

         matrixToPack.set(4, 1, value * fz_k);
         matrixToPack.set(4, 2, value * py_k);
      }
   }

   /** f_xu */
   public void getDynamicsControlGradientOfStateGradient(LIPMState hybridState, int controlVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F matrixToPack)
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

      switch (controlVariable)
      {
      case 0:
         matrixToPack.set(0, 2, fz_k  * value2);
         matrixToPack.set(3, 2, fz_k  * value);
         break;
      case 1:
         matrixToPack.set(1, 2, fz_k  * value2);
         matrixToPack.set(4, 2, fz_k  * value);
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
}
