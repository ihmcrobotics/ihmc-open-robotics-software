package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.trajectoryOptimization.DiscreteHybridDynamics;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class SimpleReactionDynamics implements DiscreteHybridDynamics<SLIPState>
{
   private double flightDuration;
   private double deltaT;
   private double deltaT2;
   private final double pendulumMass;
   private final double gravityZ;
   private static final Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
   private final Vector3D inertia = new Vector3D();

   private final ContinuousSimpleReactionDynamics continuousDynamics;
   private final DenseMatrix64F continuousDynamicsMatrix = new DenseMatrix64F(stateVectorSize / 2, 1);

   public SimpleReactionDynamics(double deltaT, double pendulumMass, double gravityZ)
   {
      this.deltaT = deltaT;
      this.deltaT2 = deltaT * deltaT;
      this.pendulumMass = pendulumMass;
      this.gravityZ = gravityZ;

      continuousDynamics = new ContinuousSimpleReactionDynamics(pendulumMass, gravityZ);

      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getX() * boxSize.getX() + boxSize.getY() * boxSize.getY());
      inertia.scale(pendulumMass / 12.0);
   }

   public void setFlightDuration(double flightDuration)
   {
      this.flightDuration = flightDuration;
   }

   @Override
   public void setTimeStepSize(double deltaT)
   {
      this.deltaT = deltaT;
      this.deltaT2 = deltaT * deltaT;
   }

   @Override
   public int getStateVectorSize()
   {
      return stateVectorSize;
   }

   @Override
   public int getControlVectorSize()
   {
      return controlVectorSize;
   }

   @Override
   public void getNextState(SLIPState hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.numRows != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (matrixToPack.numCols != 1)
         throw new RuntimeException("The state matrix size is wrong.");

      double x_k = currentState.get(x, 0);
      double y_k = currentState.get(y, 0);
      double z_k = currentState.get(z, 0);
      double xDot_k = currentState.get(xDot, 0);
      double yDot_k = currentState.get(yDot, 0);
      double zDot_k = currentState.get(zDot, 0);
      double thetaX_k = currentState.get(thetaX, 0);
      double thetaY_k = currentState.get(thetaY, 0);
      double thetaZ_k = currentState.get(thetaZ, 0);
      double thetaXDot_k = currentState.get(thetaXDot, 0);
      double thetaYDot_k = currentState.get(thetaYDot, 0);
      double thetaZDot_k = currentState.get(thetaZDot, 0);

      continuousDynamics.getDynamics(hybridState, currentState, currentControl, continuousDynamicsMatrix);

      /*
      switch (hybridState)
      {
      case FLIGHT:
         matrixToPack.set(x, 0, x_k + flightDuration * xDot_k);
         matrixToPack.set(y, 0, y_k + flightDuration * yDot_k);
         matrixToPack.set(z, 0, z_k + flightDuration * zDot_k - 0.5 * flightDuration * flightDuration * gravityZ);
         matrixToPack.set(thetaX, 0, thetaX_k + flightDuration * thetaXDot_k);
         matrixToPack.set(thetaY, 0, thetaY_k + flightDuration * thetaYDot_k);
         matrixToPack.set(thetaZ, 0, thetaZ_k + flightDuration * thetaZDot_k);

         matrixToPack.set(xDot, 0, xDot_k);
         matrixToPack.set(yDot, 0, yDot_k);
         matrixToPack.set(zDot, 0, zDot_k - flightDuration * gravityZ);
         matrixToPack.set(thetaXDot, 0, thetaXDot_k);
         matrixToPack.set(thetaYDot, 0, thetaYDot_k);
         matrixToPack.set(thetaZDot, 0, thetaZDot_k);

         break;

      case STANCE:
         double fX_k = currentControl.get(fx, 0);
         double fY_k = currentControl.get(fy, 0);
         double fZ_k = currentControl.get(fz, 0);
         double tauX_k = currentControl.get(tauX, 0);
         double tauY_k = currentControl.get(tauY, 0);
         double tauZ_k = currentControl.get(tauZ, 0);

         matrixToPack.set(x, 0, x_k + deltaT * xDot_k + 0.5 * deltaT2 * fX_k / pendulumMass);
         matrixToPack.set(y, 0, y_k + deltaT * yDot_k + 0.5 * deltaT2 * fY_k / pendulumMass);
         matrixToPack.set(z, 0, z_k + deltaT * zDot_k + 0.5 * deltaT2 * (fZ_k / pendulumMass - gravityZ));
         matrixToPack.set(thetaX, 0, thetaX_k + deltaT * thetaXDot_k + 0.5 * deltaT2 * tauX_k / inertia.getX());
         matrixToPack.set(thetaY, 0, thetaY_k + deltaT * thetaYDot_k + 0.5 * deltaT2 * tauY_k / inertia.getY());
         matrixToPack.set(thetaZ, 0, thetaZ_k + deltaT * thetaZDot_k + 0.5 * deltaT2 * tauZ_k / inertia.getZ());

         matrixToPack.set(xDot, 0, xDot_k + deltaT * fX_k / pendulumMass);
         matrixToPack.set(yDot, 0, yDot_k + deltaT * fY_k / pendulumMass);
         matrixToPack.set(zDot, 0, zDot_k + deltaT * (fZ_k / pendulumMass - gravityZ));
         matrixToPack.set(thetaXDot, 0, thetaXDot_k + deltaT * tauX_k / inertia.getX());
         matrixToPack.set(thetaYDot, 0, thetaYDot_k + deltaT * tauY_k / inertia.getY());
         matrixToPack.set(thetaZDot, 0, thetaZDot_k + deltaT * tauZ_k / inertia.getZ());

         break;
      }
      */

      matrixToPack.set(x, 0, x_k + flightDuration * xDot_k);
      matrixToPack.set(y, 0, y_k + flightDuration * yDot_k);
      matrixToPack.set(z, 0, z_k + flightDuration * zDot_k);
      matrixToPack.set(thetaX, 0, thetaX_k + flightDuration * thetaXDot_k);
      matrixToPack.set(thetaY, 0, thetaY_k + flightDuration * thetaYDot_k);
      matrixToPack.set(thetaZ, 0, thetaZ_k + flightDuration * thetaZDot_k);

      matrixToPack.set(xDot, 0, xDot_k);
      matrixToPack.set(yDot, 0, yDot_k);
      matrixToPack.set(zDot, 0, zDot_k);
      matrixToPack.set(thetaXDot, 0, thetaXDot_k);
      matrixToPack.set(thetaYDot, 0, thetaYDot_k);
      matrixToPack.set(thetaZDot, 0, thetaZDot_k);

      MatrixTools.addMatrixBlock(matrixToPack, x, 0, continuousDynamicsMatrix, x, 0, stateVectorSize / 2, 1, 0.5 * flightDuration * flightDuration);
      MatrixTools.addMatrixBlock(matrixToPack, xDot, 0, continuousDynamicsMatrix, x, 0, stateVectorSize / 2, 1, flightDuration);
   }

   @Override
   public void getDynamicsStateGradient(SLIPState hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl,
                                        DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.numRows != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (matrixToPack.numCols != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");

      matrixToPack.zero();
      CommonOps.setIdentity(matrixToPack);

      switch (hybridState)
      {
      case FLIGHT:
         matrixToPack.set(x, xDot, flightDuration);
         matrixToPack.set(y, yDot, flightDuration);
         matrixToPack.set(z, zDot, flightDuration);
         matrixToPack.set(thetaX, thetaXDot, flightDuration);
         matrixToPack.set(thetaY, thetaYDot, flightDuration);
         matrixToPack.set(thetaZ, thetaZDot, flightDuration);
         break;
      case STANCE:
         matrixToPack.set(x, xDot, deltaT);
         matrixToPack.set(y, yDot, deltaT);
         matrixToPack.set(z, zDot, deltaT);
         matrixToPack.set(thetaX, thetaXDot, deltaT);
         matrixToPack.set(thetaY, thetaYDot, deltaT);
         matrixToPack.set(thetaZ, thetaZDot, deltaT);
         break;
      }

   }

   @Override
   public void getDynamicsControlGradient(SLIPState hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl,
                                          DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.numRows != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");
      if (matrixToPack.numCols != controlVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");

      matrixToPack.zero();

      switch (hybridState)
      {
      case FLIGHT:
         matrixToPack.set(x, fx, deltaT / pendulumMass);
         matrixToPack.set(y, fy, deltaT / pendulumMass);
         matrixToPack.set(z, fz, deltaT / pendulumMass);
         matrixToPack.set(thetaX, tauX, deltaT / inertia.getX());
         matrixToPack.set(thetaY, tauY, deltaT / inertia.getY());
         matrixToPack.set(thetaZ, tauZ, deltaT / inertia.getZ());
         matrixToPack.set(xDot, fx, deltaT / pendulumMass);
         matrixToPack.set(yDot, fy, deltaT / pendulumMass);
         matrixToPack.set(zDot, fz, deltaT / pendulumMass);
         matrixToPack.set(thetaXDot, tauX, deltaT / inertia.getX());
         matrixToPack.set(thetaYDot, tauY, deltaT / inertia.getY());
         matrixToPack.set(thetaZDot, tauZ, deltaT / inertia.getZ());
         break;
      }
   }

   @Override
   public void getDynamicsStateHessian(SLIPState hybridState, int stateVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl,
                                       DenseMatrix64F matrixToPack)
   {

   }

   @Override
   public void getDynamicsControlHessian(SLIPState hybridState, int controlVariable, DenseMatrix64F currentState, DenseMatrix64F currentControl,
                                         DenseMatrix64F matrixToPack)
   {

   }

   @Override
   public void getDynamicsStateGradientOfControlGradient(SLIPState hybridState, int stateVariable, DenseMatrix64F currentState,
                                                         DenseMatrix64F currentControl, DenseMatrix64F matrixToPack)
   {

   }

   @Override
   public void getDynamicsControlGradientOfStateGradient(SLIPState hybridState, int controlVariable, DenseMatrix64F currentState,
                                                         DenseMatrix64F currentControl, DenseMatrix64F matrixToPack)
   {

   }

   @Override
   public void getContinuousAMatrix(DenseMatrix64F A)
   {

   }

   @Override
   public void getContinuousBMatrix(DenseMatrix64F B)
   {

   }
}
