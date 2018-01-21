package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.trajectoryOptimization.ContinuousHybridDynamics;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class ContinuousSimpleReactionDynamics implements ContinuousHybridDynamics<SLIPState>
{
   private final double pendulumMass;
   private final double gravityZ;
   private static final Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
   private final Vector3D inertia = new Vector3D();

   public ContinuousSimpleReactionDynamics(double pendulumMass, double gravityZ)
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
         double fX_k = currentControl.get(fx, 0);
         double fY_k = currentControl.get(fy, 0);
         double fZ_k = currentControl.get(fz, 0);
         double tauX_k = currentControl.get(tauX, 0);
         double tauY_k = currentControl.get(tauY, 0);
         double tauZ_k = currentControl.get(tauZ, 0);

         matrixToPack.set(x, 0, fX_k / pendulumMass);
         matrixToPack.set(y, 0, fY_k / pendulumMass);
         matrixToPack.set(z, 0, fZ_k / pendulumMass - gravityZ);
         matrixToPack.set(thetaX, 0, tauX_k / inertia.getX());
         matrixToPack.set(thetaY, 0, tauY_k / inertia.getY());
         matrixToPack.set(thetaZ, 0, tauZ_k / inertia.getZ());

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
         matrixToPack.set(x, fx, 1.0 / pendulumMass);
         matrixToPack.set(y, fy, 1.0 / pendulumMass);
         matrixToPack.set(z, fz, 1.0 / pendulumMass);
         matrixToPack.set(thetaX, thetaX, 1.0 / inertia.getX());
         matrixToPack.set(thetaY, thetaY, 1.0 / inertia.getY());
         matrixToPack.set(thetaZ, thetaZ, 1.0 / inertia.getZ());
      break;
      }
   }
}
