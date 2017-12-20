package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.trajectoryOptimization.ContinuousHybridDynamics;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class ContinuousSLIPDynamics implements ContinuousHybridDynamics<SLIPState>
{
   private final double pendulumMass;
   private final double gravityZ;
   private double nominalPendulumLength;
   private static final Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
   private final Vector3D inertia = new Vector3D();

   public ContinuousSLIPDynamics(double pendulumMass, double nominalPendulumLength, double gravityZ)
   {
      this.pendulumMass = pendulumMass;
      this.gravityZ = gravityZ;
      this.nominalPendulumLength = nominalPendulumLength;

      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getX() * boxSize.getX() + boxSize.getY() * boxSize.getY());
      inertia.scale(pendulumMass / 12.0);
   }

   public int getStateVectorSize()
   {
      return stateVectorSize / 2;
   }

   public int getControlVectorSize()
   {
      return controlVectorSize;
   }

   // FIXME include zF
   public void getDynamics(SLIPState hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.numRows != stateVectorSize / 2)
         throw new RuntimeException("The state matrix size is wrong.");
      if (matrixToPack.numCols != 1)
         throw new RuntimeException("The state matrix size is wrong.");

      matrixToPack.zero();

      switch (hybridState)
      {
      case FLIGHT:
         matrixToPack.set(z, 0,  -gravityZ);

         break;

      case STANCE:
         double x_k = currentState.get(x, 0);
         double y_k = currentState.get(y, 0);
         double z_k = currentState.get(z, 0);

         double fX_k = currentControl.get(fx, 0);
         double fY_k = currentControl.get(fy, 0);
         double fZ_k = currentControl.get(fz, 0);

         double xF_k = currentControl.get(xF, 0);
         double yF_k = currentControl.get(yF, 0);

         double K = currentControl.get(k, 0);

         double pendulumLength = Math.sqrt((x_k - xF_k) * (x_k - xF_k) + (y_k - yF_k) * (y_k - yF_k) + z_k * z_k);

         matrixToPack.set(x, 0, K / pendulumMass * (nominalPendulumLength / pendulumLength - 1.0) * (x_k - xF_k));
         matrixToPack.set(y, 0, K / pendulumMass * (nominalPendulumLength / pendulumLength - 1.0) * (y_k - yF_k));
         matrixToPack.set(z, 0, K / pendulumMass * (nominalPendulumLength / pendulumLength - 1.0) * z_k - gravityZ);
         matrixToPack.set(thetaX, 0, 1.0 / inertia.getX() * (-z_k * fY_k + (y_k - yF_k) * fZ_k));
         matrixToPack.set(thetaY, 0, 1.0 / inertia.getY() * (z_k * fX_k - (x_k - xF_k) * fZ_k));
         matrixToPack.set(thetaZ, 0, 1.0 / inertia.getZ() * (-(y_k - yF_k) * fX_k + (x_k - xF_k) * fY_k));

         break;
      }
   }

   public void getDynamicsStateGradient(SLIPState hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl,
                                        DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.numRows != stateVectorSize / 2)
         throw new RuntimeException("The state matrix size is wrong.");
      if (matrixToPack.numCols != stateVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");

      matrixToPack.zero();

      switch (hybridState)
      {
      case STANCE:
         double x_k = currentState.get(x, 0);
         double y_k = currentState.get(y, 0);
         double z_k = currentState.get(z, 0);

         double fX_k = currentControl.get(fx, 0);
         double fY_k = currentControl.get(fy, 0);
         double fZ_k = currentControl.get(fz, 0);

         double xF_k = currentControl.get(xF, 0);
         double yF_k = currentControl.get(yF, 0);

         double K = currentControl.get(k, 0);

         double pendulumLength = Math.sqrt((x_k - xF_k) * (x_k - xF_k) + (y_k - yF_k) * (y_k - yF_k) + z_k * z_k);

         matrixToPack.set(x, x, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * Math.pow(x_k - xF_k, 2.0)
               + nominalPendulumLength / pendulumLength - 1.0));
         matrixToPack.set(x, y, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * (x_k - xF_k) * (y_k - yF_k)));
         matrixToPack.set(x, z, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * (x_k - xF_k) * z_k));

         matrixToPack.set(y, x, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * (x_k - xF_k) * (y_k - yF_k)));
         matrixToPack.set(y, y, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * Math.pow(y_k - yF_k, 2.0)
               + nominalPendulumLength / pendulumLength - 1.0));
         matrixToPack.set(y, z, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * (y_k - yF_k) * z_k));

         matrixToPack.set(z, x, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * (x_k - xF_k) * z_k));
         matrixToPack.set(z, y, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * (y_k - yF_k) * z_k));
         matrixToPack.set(z, z, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * Math.pow(z_k, 2.0)
               + nominalPendulumLength / pendulumLength - 1.0));

         matrixToPack.set(thetaX, y, 1.0 / inertia.getX() * fZ_k);
         matrixToPack.set(thetaX, z, 1.0 / inertia.getX() * -fY_k);

         matrixToPack.set(thetaY, x, 1.0 / inertia.getY() * -fZ_k);
         matrixToPack.set(thetaY, z, 1.0 / inertia.getY() * fX_k);

         matrixToPack.set(thetaZ, x, 1.0 / inertia.getZ() * fY_k);
         matrixToPack.set(thetaZ, y, 1.0 / inertia.getZ() * -fX_k);
      }
   }

   public void getDynamicsControlGradient(SLIPState hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl,
                                          DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.numRows != stateVectorSize / 2)
         throw new RuntimeException("The state matrix size is wrong.");
      if (matrixToPack.numCols != controlVectorSize)
         throw new RuntimeException("The state matrix size is wrong.");

      matrixToPack.zero();

      switch (hybridState)
      {
      case STANCE:
         double x_k = currentState.get(x, 0);
         double y_k = currentState.get(y, 0);
         double z_k = currentState.get(z, 0);

         double xF_k = currentControl.get(xF, 0);
         double yF_k = currentControl.get(yF, 0);

         double K = currentControl.get(k, 0);
         double pendulumLength = Math.sqrt((x_k - xF_k) * (x_k - xF_k) + (y_k - yF_k) * (y_k - yF_k) + z_k * z_k);

         matrixToPack.set(x, xF, K / pendulumMass * (nominalPendulumLength / Math.pow(pendulumLength, 3.0) * Math.pow(x_k - xF_k, 2.0)
               - nominalPendulumLength / pendulumLength + 1.0));
         matrixToPack.set(x, yF, K / pendulumMass * (nominalPendulumLength / Math.pow(pendulumLength, 3.0) * (x_k - xF_k) * (y_k - yF_k)));
         matrixToPack.set(x, k, 1.0 / pendulumMass * (nominalPendulumLength / pendulumLength - 1.0) * (x_k - xF_k));

         matrixToPack.set(y, xF, K / pendulumMass * (nominalPendulumLength / Math.pow(pendulumLength, 3.0) * (x_k - xF_k) * (y_k - yF_k)));
         matrixToPack.set(y, yF, K / pendulumMass * (nominalPendulumLength / Math.pow(pendulumLength, 3.0) * Math.pow(y_k - yF_k, 2.0)
               - nominalPendulumLength / pendulumLength + 1.0));
         matrixToPack.set(y, k, 1.0 / pendulumMass * (nominalPendulumLength / pendulumLength - 1.0) * (y_k - yF_k));

         matrixToPack.set(z, xF, K / pendulumMass * (nominalPendulumLength / Math.pow(pendulumLength, 3.0) * (x_k - xF_k) * z_k));
         matrixToPack.set(z, yF, K / pendulumMass * (nominalPendulumLength / Math.pow(pendulumLength, 3.0) * (y_k - yF_k) * z_k));
         matrixToPack.set(z, k, 1.0 / pendulumMass * (nominalPendulumLength / pendulumLength - 1.0) * z_k);

         matrixToPack.set(thetaX, fy, 1.0 / inertia.getX() * -z_k);
         matrixToPack.set(thetaX, fz, 1.0 / inertia.getX() * (y_k - yF_k));

         matrixToPack.set(thetaY, fx, 1.0 / inertia.getY() * z_k);
         matrixToPack.set(thetaY, fz, 1.0 / inertia.getY() * - (x_k - xF_k));

         matrixToPack.set(thetaZ, fx, 1.0 / inertia.getZ() * -(y_k - yF_k));
         matrixToPack.set(thetaZ, fy, 1.0 / inertia.getZ() * (x_k - xF_k));
      }
   }
}
