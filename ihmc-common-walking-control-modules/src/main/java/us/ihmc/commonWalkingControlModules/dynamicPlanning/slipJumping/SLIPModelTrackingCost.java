package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.trajectoryOptimization.LQCostFunction;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class SLIPModelTrackingCost implements LQCostFunction<SLIPState>
{
   static double qFX = 1.0e2;
   static double qFY = 1.0e2;
   static double qFZ = 1.0e2;
   static double qTauX = 1.0e2;
   static double qTauY = 1.0e2;
   static double qTauZ = 1.0e2;

   private final DenseMatrix64F Q = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize / 2);
   private final ContinuousSimpleReactionDynamics simpleReactionDynamics;
   private final ContinuousSLIPDynamics slipDynamics;

   private final DenseMatrix64F dynamicsError = new DenseMatrix64F(stateVectorSize / 2, 1);
   private final DenseMatrix64F stateGradientError = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);
   private final DenseMatrix64F controlGradientError = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);

   private final DenseMatrix64F slipFunction = new DenseMatrix64F(stateVectorSize / 2, 1);
   private final DenseMatrix64F simpleReactionFunction = new DenseMatrix64F(stateVectorSize / 2, 1);

   private final DenseMatrix64F slipStateGradient = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);
   private final DenseMatrix64F simpleStateGradient = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);

   private final DenseMatrix64F slipControlGradient = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);
   private final DenseMatrix64F simpleControlGradient = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);

   private final DenseMatrix64F scalarCost = new DenseMatrix64F(1, 1);

   private final double pendulumMass;
   private static final Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
   private final Vector3D inertia = new Vector3D();

   private final double nominalPendulumLength;

   public SLIPModelTrackingCost(double pendulumMass, double nominalPendulumLength, double gravityZ)
   {
      this.pendulumMass = pendulumMass;
      this.nominalPendulumLength = nominalPendulumLength;
      simpleReactionDynamics = new ContinuousSimpleReactionDynamics(pendulumMass, gravityZ);
      slipDynamics = new ContinuousSLIPDynamics(pendulumMass, nominalPendulumLength, gravityZ);

      Q.set(x, x, qFX);
      Q.set(y, y, qFY);
      Q.set(z, z, qFZ);
      Q.set(tauX, tauX, qTauX);
      Q.set(tauY, tauY, qTauY);
      Q.set(tauZ, tauZ, qTauZ);

      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getX() * boxSize.getX() + boxSize.getY() * boxSize.getY());
      inertia.scale(pendulumMass / 12.0);
   }


   @Override
   public double getCost(SLIPState hybridState, DenseMatrix64F controlVector, DenseMatrix64F stateVector)
   {
      simpleReactionDynamics.getDynamics(hybridState, stateVector, controlVector, simpleReactionFunction);
      slipDynamics.getDynamics(hybridState, stateVector, controlVector, slipFunction);

      CommonOps.subtract(simpleReactionFunction, slipFunction, dynamicsError);
      MatrixTools.scaleRow(pendulumMass, x, dynamicsError);
      MatrixTools.scaleRow(pendulumMass, y, dynamicsError);
      MatrixTools.scaleRow(pendulumMass, z, dynamicsError);
      MatrixTools.scaleRow(inertia.getX(), thetaX, dynamicsError);
      MatrixTools.scaleRow(inertia.getY(), thetaY, dynamicsError);
      MatrixTools.scaleRow(inertia.getZ(), thetaZ, dynamicsError);

      multQuad(dynamicsError, Q, dynamicsError, scalarCost);

      return scalarCost.get(0);
   }

   @Override
   public void getCostStateGradient(SLIPState hybridState, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.getNumRows() != stateVectorSize && matrixToPack.getNumCols() != 1)
         throw new RuntimeException("Matrix control gradient is the improper size.");

      simpleReactionDynamics.getDynamics(hybridState, stateVector, controlVector, simpleReactionFunction);
      slipDynamics.getDynamics(hybridState, stateVector, controlVector, slipFunction);

      simpleReactionDynamics.getDynamicsStateGradient(hybridState, stateVector, controlVector, simpleStateGradient);
      slipDynamics.getDynamicsStateGradient(hybridState, stateVector, controlVector, slipStateGradient);

      CommonOps.subtract(simpleReactionFunction, slipFunction, dynamicsError);
      CommonOps.subtract(simpleStateGradient, slipStateGradient, stateGradientError);

      MatrixTools.scaleRow(pendulumMass, x, dynamicsError);
      MatrixTools.scaleRow(pendulumMass, y, dynamicsError);
      MatrixTools.scaleRow(pendulumMass, z, dynamicsError);
      MatrixTools.scaleRow(inertia.getX(), thetaX, dynamicsError);
      MatrixTools.scaleRow(inertia.getY(), thetaY, dynamicsError);
      MatrixTools.scaleRow(inertia.getZ(), thetaZ, dynamicsError);

      MatrixTools.scaleRow(pendulumMass, x, stateGradientError);
      MatrixTools.scaleRow(pendulumMass, y, stateGradientError);
      MatrixTools.scaleRow(pendulumMass, z, stateGradientError);
      MatrixTools.scaleRow(inertia.getX(), thetaX, stateGradientError);
      MatrixTools.scaleRow(inertia.getY(), thetaY, stateGradientError);
      MatrixTools.scaleRow(inertia.getZ(), thetaZ, stateGradientError);

      multQuad(2.0, stateGradientError, Q, dynamicsError, matrixToPack);
   }

   @Override
   public void getCostControlGradient(SLIPState hybridState, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.getNumRows() != controlVectorSize && matrixToPack.getNumCols() != 1)
         throw new RuntimeException("Matrix control gradient is the improper size.");

      simpleReactionDynamics.getDynamics(hybridState, stateVector, controlVector, simpleReactionFunction);
      slipDynamics.getDynamics(hybridState, stateVector, controlVector, slipFunction);

      simpleReactionDynamics.getDynamicsControlGradient(hybridState, stateVector, controlVector, simpleControlGradient);
      slipDynamics.getDynamicsControlGradient(hybridState, stateVector, controlVector, slipControlGradient);

      CommonOps.subtract(simpleReactionFunction, slipFunction, dynamicsError);
      CommonOps.subtract(simpleControlGradient, slipControlGradient, controlGradientError);

      MatrixTools.scaleRow(pendulumMass, x, dynamicsError);
      MatrixTools.scaleRow(pendulumMass, y, dynamicsError);
      MatrixTools.scaleRow(pendulumMass, z, dynamicsError);
      MatrixTools.scaleRow(inertia.getX(), thetaX, dynamicsError);
      MatrixTools.scaleRow(inertia.getY(), thetaY, dynamicsError);
      MatrixTools.scaleRow(inertia.getZ(), thetaZ, dynamicsError);

      MatrixTools.scaleRow(pendulumMass, x, controlGradientError);
      MatrixTools.scaleRow(pendulumMass, y, controlGradientError);
      MatrixTools.scaleRow(pendulumMass, z, controlGradientError);
      MatrixTools.scaleRow(inertia.getX(), thetaX, controlGradientError);
      MatrixTools.scaleRow(inertia.getY(), thetaY, controlGradientError);
      MatrixTools.scaleRow(inertia.getZ(), thetaZ, controlGradientError);

      multQuad(2.0, controlGradientError, Q, dynamicsError, matrixToPack);
   }

   @Override
   public void getCostStateHessian(SLIPState hybridState, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.getNumRows() != stateVectorSize)
         throw new RuntimeException("Matrix state hessian has improper number of rows.");
      if (matrixToPack.getNumCols() != stateVectorSize)
         throw new RuntimeException("Matrix state hessian has improper number of columns.");

      switch (hybridState)
      {
      case STANCE:
         double x_k = stateVector.get(x);
         double y_k = stateVector.get(y);
         double z_k = stateVector.get(z);

         double xF_k = controlVector.get(xF);
         double yF_k = controlVector.get(yF);
         double zF_k = 0.0;

         double relativeX = x_k - xF_k;
         double relativeY = y_k - yF_k;
         double relativeZ = z_k - zF_k;

         double fX_k = controlVector.get(fx);
         double fY_k = controlVector.get(fy);
         double fZ_k = controlVector.get(fz);

         double K_k = controlVector.get(k);

         double currentLength = Math.sqrt(relativeX * relativeX + relativeY * relativeY + relativeZ * relativeZ);

         double normalizedSpringForce = K_k * (nominalPendulumLength / currentLength - 1.0);

         double xForceDifference = fX_k - normalizedSpringForce * relativeX;
         double yForceDifference = fY_k - normalizedSpringForce * relativeY;
         double zForceDifference = fZ_k - normalizedSpringForce * relativeZ;

         double qFx = Q.get(x, x);
         double qFy = Q.get(y, y);
         double qFz = Q.get(z, z);
         double qTx = Q.get(tauX, tauX);
         double qTy = Q.get(tauY, tauY);
         double qTz = Q.get(tauZ, tauZ);

         double dfXdx = K_k * (nominalPendulumLength * Math.pow(currentLength, -3.0) * relativeX * relativeX - nominalPendulumLength / currentLength + 1.0);
         double dfYdx = K_k * nominalPendulumLength * Math.pow(currentLength, -3.0) * relativeX * relativeY;
         double dfZdx = K_k * nominalPendulumLength * Math.pow(currentLength, -3.0) * relativeX * relativeZ;

         double dfXdy = K_k * nominalPendulumLength * Math.pow(currentLength, -3.0) * relativeX * relativeY;
         double dfYdy = K_k * (nominalPendulumLength * Math.pow(currentLength, -3.0) * relativeY * relativeY - nominalPendulumLength / currentLength + 1.0);
         double dfZdy = K_k * nominalPendulumLength * Math.pow(currentLength, -3.0) * relativeY * relativeZ;

         double dfXdz = K_k * nominalPendulumLength * Math.pow(currentLength, -3.0) * relativeX * relativeZ;
         double dfYdz = K_k * nominalPendulumLength * Math.pow(currentLength, -3.0) * relativeY * relativeZ;
         double dfZdz = K_k * (nominalPendulumLength * Math.pow(currentLength, -3.0) * relativeZ * relativeZ - nominalPendulumLength / currentLength + 1.0);

         double l3 = Math.pow(currentLength, -3.0);
         double l5 = Math.pow(currentLength, -5.0);

         double dfXdxdx = 3.0 * K_k * nominalPendulumLength * relativeX * l3 - 3.0 * K_k * nominalPendulumLength * Math.pow(relativeX, 3.0) * l5;
         double dfYdxdx = K_k * nominalPendulumLength * relativeY * l3 - 3.0 * K_k * nominalPendulumLength * relativeX * relativeX * relativeY * l5;
         double dfZdxdx = K_k * nominalPendulumLength * relativeZ * l3 - 3.0 * K_k * nominalPendulumLength * relativeX * relativeX * relativeZ * l5;

         double dfXdxdy = K_k * nominalPendulumLength * relativeY * l3 - 3.0 * K_k * nominalPendulumLength * relativeX * relativeX * relativeY * l5;
         double dfYdxdy = K_k * nominalPendulumLength * relativeX * l3 - 3.0 * K_k * nominalPendulumLength * relativeX * relativeY * relativeY * l5;
         double dfZdxdy = -3.0 * K_k * nominalPendulumLength * relativeX * relativeY * relativeZ * l5;

         double dfXdxdz = K_k * nominalPendulumLength * relativeZ * l3 - 3.0 * K_k * nominalPendulumLength * relativeX * relativeX * relativeZ * l5;
         double dfYdxdz = -3.0 * K_k * nominalPendulumLength * relativeX * relativeY * relativeZ * l5;
         double dfZdxdz = K_k * nominalPendulumLength * relativeX * l3 - 3.0 * K_k * nominalPendulumLength * relativeX * relativeZ * relativeZ * l5;

         double dfXdydy = K_k * nominalPendulumLength * relativeX * l3 - 3.0 * K_k * nominalPendulumLength * relativeX * relativeY * relativeY * l5;
         double dfYdydy = 3.0 * K_k * nominalPendulumLength * relativeY * l3 - 3.0 * K_k * nominalPendulumLength * Math.pow(relativeY, 3.0) * l5;
         double dfZdydy = K_k * nominalPendulumLength * relativeZ * l3 - 3.0 * K_k * nominalPendulumLength * relativeY * relativeY * relativeZ * l5;

         double dfXdydz = -3.0 * K_k * nominalPendulumLength * relativeX * relativeY * relativeZ * l5;
         double dfYdydz = K_k * nominalPendulumLength * relativeZ * l3 - 3.0 * K_k * nominalPendulumLength * relativeY * relativeY * relativeZ * l5;
         double dfZdydz = K_k * nominalPendulumLength * relativeY * l3 - 3.0 * K_k * nominalPendulumLength * relativeY * relativeZ * relativeZ * l5;

         double dfXdzdz = K_k * nominalPendulumLength * relativeX * l3 - 3.0 * K_k * nominalPendulumLength * relativeX * relativeZ * relativeZ * l5;
         double dfYdzdz = K_k * nominalPendulumLength * relativeY * l3 - 3.0 * K_k * nominalPendulumLength * relativeY * relativeZ * relativeX * l5;
         double dfZdzdz = 3.0 * K_k * nominalPendulumLength * relativeZ * l3 - 3.0 * K_k * nominalPendulumLength * Math.pow(relativeZ, 3.0) * l5;

         double Jxx = 2.0 * qFx * (dfXdxdx * xForceDifference + dfXdx * dfXdx);
         Jxx += 2.0 * qFy * (dfYdxdx * yForceDifference + dfYdx * dfYdx);
         Jxx += 2.0 * qFz * (dfZdxdx * zForceDifference + dfZdx * dfZdx);
         Jxx += 2.0 * qTy * fZ_k * fZ_k + qTz * qTz * fY_k * fY_k;

         double Jxy = 2.0 * qFx * (dfXdxdy * xForceDifference + dfXdx * dfXdy);
         Jxy += 2.0 * qFy * (dfYdxdy * yForceDifference + dfYdx * dfYdy);
         Jxy += 2.0 * qFz * (dfZdxdy * zForceDifference + dfZdx * dfZdy);
         Jxy -= 2.0 * qTz * fY_k * fX_k;

         double Jxz = 2.0 * qFx * (dfXdxdz * xForceDifference + dfXdx * dfXdz);
         Jxz += 2.0 * qFy * (dfYdxdz * yForceDifference + dfYdx * dfYdz);
         Jxz += 2.0 * qFz * (dfZdxdz * zForceDifference + dfZdx * dfZdz);
         Jxz -= 2.0 * qTy * fZ_k * fX_k;

         double Jyy = 2.0 * qFx * (dfXdydy * xForceDifference + dfXdy * dfXdy);
         Jyy += 2.0 * qFy * (dfYdydy * yForceDifference + dfYdy * dfYdy);
         Jyy += 2.0 * qFz * (dfZdydy * zForceDifference + dfZdy * dfZdy);
         Jyy += 2.0 * qTz * fX_k * fX_k - qTx * qTx * fZ_k * fZ_k;

         double Jyz = 2.0 * qFx * (dfXdydz * xForceDifference + dfXdy * dfXdz);
         Jyz += 2.0 * qFy * (dfYdydz * yForceDifference + dfYdy * dfYdz);
         Jyz += 2.0 * qFz * (dfZdydz * zForceDifference + dfZdy * dfZdz);
         Jyz -= 2.0 * qTx * fX_k * fY_k;

         double Jzz = 2.0 * qFx * (dfXdzdz * xForceDifference + dfXdz * dfXdz);
         Jzz += 2.0 * qFy * (dfYdzdz * yForceDifference + dfYdz * dfYdz);
         Jzz += 2.0 * qFz * (dfZdzdz * zForceDifference + dfZdz * dfZdz);
         Jzz += 2.0 * qTx * fY_k * fY_k - 2.0 * qTy * fX_k * fX_k;

         matrixToPack.set(x, x, Jxx);
         matrixToPack.set(x, y, Jxy);
         matrixToPack.set(x, z, Jxz);

         matrixToPack.set(y, x, Jxy);
         matrixToPack.set(y, y, Jyy);
         matrixToPack.set(y, z, Jyz);

         matrixToPack.set(z, x, Jxz);
         matrixToPack.set(z, y, Jyz);
         matrixToPack.set(z, z, Jzz);

         break;


      case FLIGHT:





         break;
      }

   }

   @Override
   public void getCostControlHessian(SLIPState hybridState, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {

   }

   @Override
   public void getCostStateGradientOfControlGradient(SLIPState hybridState, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {

   }

   @Override
   public void getCostControlGradientOfStateGradient(SLIPState hybridState, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {

   }

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);

   private void multQuad(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F c, DenseMatrix64F d)
   {
      tempMatrix.reshape(a.numCols, b.numCols);
      CommonOps.multTransA(a, b, tempMatrix);
      CommonOps.mult(tempMatrix, c, d);
   }


   private void multQuad(double alpha, DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F c, DenseMatrix64F d)
   {
      tempMatrix.reshape(a.numCols, b.numCols);
      CommonOps.multTransA(alpha, a, b, tempMatrix);
      CommonOps.mult(tempMatrix, c, d);
   }
}
