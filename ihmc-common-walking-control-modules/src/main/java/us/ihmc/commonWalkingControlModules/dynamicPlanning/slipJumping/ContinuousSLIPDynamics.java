package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.trajectoryOptimization.ContinuousHybridDynamics;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class ContinuousSLIPDynamics implements ContinuousHybridDynamics<SLIPState>
{
   private final double pendulumMass;
   private final double gravityZ;
   private static final Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
   private final Vector3D inertia = new Vector3D();

   public ContinuousSLIPDynamics(double pendulumMass, double gravityZ)
   {
      this.pendulumMass = pendulumMass;
      this.gravityZ = gravityZ;

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

   @Override
   public void getDynamics(SLIPState hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F constants, DenseMatrix64F matrixToPack)
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

         double zF_k = constants.get(zF, 0);
         double nominalPendulumLength = constants.get(nominalLength, 0);

         double K = currentControl.get(k, 0);

         double relativeX = x_k - xF_k;
         double relativeY = y_k - yF_k;
         double relativeZ = z_k - zF_k;

         double pendulumLength = Math.sqrt(relativeX * relativeX + relativeY * relativeY + relativeZ * relativeZ);

         matrixToPack.set(x, 0, K / pendulumMass * (nominalPendulumLength / pendulumLength - 1.0) * relativeX);
         matrixToPack.set(y, 0, K / pendulumMass * (nominalPendulumLength / pendulumLength - 1.0) * relativeY);
         matrixToPack.set(z, 0, K / pendulumMass * (nominalPendulumLength / pendulumLength - 1.0) * relativeZ - gravityZ);
         matrixToPack.set(thetaX, 0, 1.0 / inertia.getX() * (-relativeZ * fY_k + relativeY * fZ_k));
         matrixToPack.set(thetaY, 0, 1.0 / inertia.getY() * (relativeZ * fX_k - relativeX * fZ_k));
         matrixToPack.set(thetaZ, 0, 1.0 / inertia.getZ() * (-relativeY * fX_k + relativeX * fY_k));

         break;
      }
   }

   @Override
   public void getDynamicsStateGradient(SLIPState hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F constants,
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

         double zF_k = constants.get(zF, 0);
         double nominalPendulumLength = constants.get(nominalLength, 0);

         double relativeX = x_k - xF_k;
         double relativeY = y_k - yF_k;
         double relativeZ = z_k - zF_k;

         double pendulumLength = Math.sqrt(relativeX * relativeX + relativeY * relativeY + relativeZ * relativeZ);


         matrixToPack.set(x, x, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * relativeX * relativeX
               + nominalPendulumLength / pendulumLength - 1.0));
         matrixToPack.set(x, y, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * relativeX * relativeY));
         matrixToPack.set(x, z, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * relativeX * relativeZ));

         matrixToPack.set(y, x, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * relativeX * relativeY));
         matrixToPack.set(y, y, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * relativeY * relativeY
               + nominalPendulumLength / pendulumLength - 1.0));
         matrixToPack.set(y, z, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * relativeY * relativeZ));

         matrixToPack.set(z, x, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * relativeX * relativeZ));
         matrixToPack.set(z, y, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * relativeY * relativeZ));
         matrixToPack.set(z, z, K / pendulumMass * (-nominalPendulumLength / Math.pow(pendulumLength, 3.0) * relativeZ * relativeZ
               + nominalPendulumLength / pendulumLength - 1.0));

         matrixToPack.set(thetaX, y, 1.0 / inertia.getX() * fZ_k);
         matrixToPack.set(thetaX, z, 1.0 / inertia.getX() * -fY_k);

         matrixToPack.set(thetaY, x, 1.0 / inertia.getY() * -fZ_k);
         matrixToPack.set(thetaY, z, 1.0 / inertia.getY() * fX_k);

         matrixToPack.set(thetaZ, x, 1.0 / inertia.getZ() * fY_k);
         matrixToPack.set(thetaZ, y, 1.0 / inertia.getZ() * -fX_k);
      }
   }

   @Override
   public void getDynamicsControlGradient(SLIPState hybridState, DenseMatrix64F currentState, DenseMatrix64F currentControl, DenseMatrix64F constants,
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

         double fx_k = currentControl.get(fx, 0);
         double fy_k = currentControl.get(fy, 0);
         double fz_k = currentControl.get(fz, 0);

         double K = currentControl.get(k, 0);

         double zF_k = constants.get(zF, 0);
         double nominalPendulumLength = constants.get(nominalLength);

         double relativeX = x_k - xF_k;
         double relativeY = y_k - yF_k;
         double relativeZ = z_k - zF_k;

         double pendulumLength = Math.sqrt(relativeX * relativeX + relativeY * relativeY + relativeZ * relativeZ);

         matrixToPack.set(x, xF, K / pendulumMass * (nominalPendulumLength / Math.pow(pendulumLength, 3.0) * relativeX * relativeX
               - nominalPendulumLength / pendulumLength + 1.0));
         matrixToPack.set(x, yF, K / pendulumMass * (nominalPendulumLength / Math.pow(pendulumLength, 3.0) * relativeX * relativeY));
         matrixToPack.set(x, k, 1.0 / pendulumMass * (nominalPendulumLength / pendulumLength - 1.0) * relativeX);

         matrixToPack.set(y, xF, K / pendulumMass * (nominalPendulumLength / Math.pow(pendulumLength, 3.0) * relativeX * relativeY));
         matrixToPack.set(y, yF, K / pendulumMass * (nominalPendulumLength / Math.pow(pendulumLength, 3.0) * relativeY * relativeY
               - nominalPendulumLength / pendulumLength + 1.0));
         matrixToPack.set(y, k, 1.0 / pendulumMass * (nominalPendulumLength / pendulumLength - 1.0) * relativeY);

         matrixToPack.set(z, xF, K / pendulumMass * (nominalPendulumLength / Math.pow(pendulumLength, 3.0) * relativeX * relativeZ));
         matrixToPack.set(z, yF, K / pendulumMass * (nominalPendulumLength / Math.pow(pendulumLength, 3.0) * relativeY * relativeZ));
         matrixToPack.set(z, k, 1.0 / pendulumMass * (nominalPendulumLength / pendulumLength - 1.0) * relativeZ);

         matrixToPack.set(thetaX, fy, 1.0 / inertia.getX() * -relativeZ);
         matrixToPack.set(thetaX, fz, 1.0 / inertia.getX() * relativeY);
         matrixToPack.set(thetaX, yF, 1.0 / inertia.getX() * -fz_k);

         matrixToPack.set(thetaY, fx, 1.0 / inertia.getY() * relativeZ);
         matrixToPack.set(thetaY, fz, 1.0 / inertia.getY() * -relativeX);
         matrixToPack.set(thetaY, xF, 1.0 / inertia.getY() * fz_k);

         matrixToPack.set(thetaZ, fx, 1.0 / inertia.getZ() * -relativeY);
         matrixToPack.set(thetaZ, fy, 1.0 / inertia.getZ() * relativeX);
         matrixToPack.set(thetaZ, xF, 1.0 / inertia.getZ() * -fy_k);
         matrixToPack.set(thetaZ, yF, 1.0 / inertia.getZ() * fx_k);
      }
   }
}
