package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.costs;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.ContinuousSLIPDynamics;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.ContinuousSimpleReactionDynamics;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.trajectoryOptimization.LQCostFunction;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class SLIPModelForceTrackingCost implements LQCostFunction<SLIPState>
{
   static double qFX = 5.0;
   static double qFY = 5.0;
   static double qFZ = 5.0;
   static double qTauX = 1.0;
   static double qTauY = 1.0;
   static double qTauZ = 1.0;

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

   public SLIPModelForceTrackingCost(double pendulumMass, double gravityZ)
   {
      this.pendulumMass = pendulumMass;
      simpleReactionDynamics = new ContinuousSimpleReactionDynamics(pendulumMass, gravityZ);
      slipDynamics = new ContinuousSLIPDynamics(pendulumMass, gravityZ);

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
   public double getCost(SLIPState hybridState, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F constants)
   {
      simpleReactionDynamics.getDynamics(hybridState, stateVector, controlVector, constants, simpleReactionFunction);
      slipDynamics.getDynamics(hybridState, stateVector, controlVector, constants, slipFunction);

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
   public void getCostStateGradient(SLIPState hybridState, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F constants, DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.getNumRows() != stateVectorSize && matrixToPack.getNumCols() != 1)
         throw new RuntimeException("Matrix control gradient is the improper size.");

      simpleReactionDynamics.getDynamics(hybridState, stateVector, controlVector, constants, simpleReactionFunction);
      slipDynamics.getDynamics(hybridState, stateVector, controlVector, constants, slipFunction);

      simpleReactionDynamics.getDynamicsStateGradient(hybridState, stateVector, controlVector, constants, simpleStateGradient);
      slipDynamics.getDynamicsStateGradient(hybridState, stateVector, controlVector, constants, slipStateGradient);

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
   public void getCostControlGradient(SLIPState hybridState, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F constants,
                                      DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.getNumRows() != controlVectorSize && matrixToPack.getNumCols() != 1)
         throw new RuntimeException("Matrix control gradient is the improper size.");

      simpleReactionDynamics.getDynamics(hybridState, stateVector, controlVector, constants, simpleReactionFunction);
      slipDynamics.getDynamics(hybridState, stateVector, controlVector, constants, slipFunction);

      simpleReactionDynamics.getDynamicsControlGradient(hybridState, stateVector, controlVector, constants, simpleControlGradient);
      slipDynamics.getDynamicsControlGradient(hybridState, stateVector, controlVector, constants, slipControlGradient);

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
   public void getCostStateHessian(SLIPState hybridState, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F constants,
                                   DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.getNumRows() != stateVectorSize)
         throw new RuntimeException("Matrix state hessian has improper number of rows.");
      if (matrixToPack.getNumCols() != stateVectorSize)
         throw new RuntimeException("Matrix state hessian has improper number of columns.");

      matrixToPack.zero();

      switch (hybridState)
      {
      case STANCE:
         double x_k = stateVector.get(x);
         double y_k = stateVector.get(y);
         double z_k = stateVector.get(z);

         double xF_k = controlVector.get(xF);
         double yF_k = controlVector.get(yF);

         double zF_k = constants.get(zF);

         double relativeX = x_k - xF_k;
         double relativeY = y_k - yF_k;
         double relativeZ = z_k - zF_k;

         double fX_k = controlVector.get(fx);
         double fY_k = controlVector.get(fy);
         double fZ_k = controlVector.get(fz);

         double K_k = controlVector.get(k);

         double currentLength = Math.sqrt(relativeX * relativeX + relativeY * relativeY + relativeZ * relativeZ);
         double l3 = Math.pow(currentLength, -3.0);
         double l5 = Math.pow(currentLength, -5.0);

         double nominalPendulumLength = constants.get(nominalLength);
         double normalizedSpringCompression = nominalPendulumLength / currentLength - 1.0;
         double normalizedSpringForce = K_k * normalizedSpringCompression;

         double xForceDifference = fX_k - normalizedSpringForce * relativeX;
         double yForceDifference = fY_k - normalizedSpringForce * relativeY;
         double zForceDifference = fZ_k - normalizedSpringForce * relativeZ;

         double qFx = Q.get(x, x);
         double qFy = Q.get(y, y);
         double qFz = Q.get(z, z);
         double qTx = Q.get(tauX, tauX);
         double qTy = Q.get(tauY, tauY);
         double qTz = Q.get(tauZ, tauZ);

         double dfXdx = K_k * (nominalPendulumLength * l3 * relativeX * relativeX - normalizedSpringCompression);
         double dfYdx = K_k * nominalPendulumLength * l3 * relativeX * relativeY;
         double dfZdx = K_k * nominalPendulumLength * l3 * relativeX * relativeZ;

         double dfXdy = K_k * nominalPendulumLength * l3 * relativeX * relativeY;
         double dfYdy = K_k * (nominalPendulumLength * l3 * relativeY * relativeY - normalizedSpringCompression);
         double dfZdy = K_k * nominalPendulumLength * l3 * relativeY * relativeZ;

         double dfXdz = K_k * nominalPendulumLength * l3 * relativeX * relativeZ;
         double dfYdz = K_k * nominalPendulumLength * l3 * relativeY * relativeZ;
         double dfZdz = K_k * (nominalPendulumLength * l3 * relativeZ * relativeZ - normalizedSpringCompression);

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
         double dfYdzdz = K_k * nominalPendulumLength * relativeY * l3 - 3.0 * K_k * nominalPendulumLength * relativeY * relativeZ * relativeZ * l5;
         double dfZdzdz = 3.0 * K_k * nominalPendulumLength * relativeZ * l3 - 3.0 * K_k * nominalPendulumLength * Math.pow(relativeZ, 3.0) * l5;

         double Jxx = 2.0 * qFx * (dfXdxdx * xForceDifference + dfXdx * dfXdx);
         Jxx += 2.0 * qFy * (dfYdxdx * yForceDifference + dfYdx * dfYdx);
         Jxx += 2.0 * qFz * (dfZdxdx * zForceDifference + dfZdx * dfZdx);
         Jxx += 2.0 * qTy * fZ_k * fZ_k + 2.0 * qTz * fY_k * fY_k;

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
         Jyy += 2.0 * qTz * fX_k * fX_k + 2.0 * qTx * fZ_k * fZ_k;

         double Jyz = 2.0 * qFx * (dfXdydz * xForceDifference + dfXdy * dfXdz);
         Jyz += 2.0 * qFy * (dfYdydz * yForceDifference + dfYdy * dfYdz);
         Jyz += 2.0 * qFz * (dfZdydz * zForceDifference + dfZdy * dfZdz);
         Jyz -= 2.0 * qTx * fZ_k * fY_k;

         double Jzz = 2.0 * qFx * (dfXdzdz * xForceDifference + dfXdz * dfXdz);
         Jzz += 2.0 * qFy * (dfYdzdz * yForceDifference + dfYdz * dfYdz);
         Jzz += 2.0 * qFz * (dfZdzdz * zForceDifference + dfZdz * dfZdz);
         Jzz += 2.0 * qTx * fY_k * fY_k + 2.0 * qTy * fX_k * fX_k;

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
   public void getCostControlHessian(SLIPState hybridState, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F constants,
                                     DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.getNumRows() != controlVectorSize)
         throw new RuntimeException("Matrix state hessian has improper number of rows.");
      if (matrixToPack.getNumCols() != controlVectorSize)
         throw new RuntimeException("Matrix state hessian has improper number of columns.");

      matrixToPack.zero();

      switch (hybridState)
      {
      case STANCE:
         double x_k = stateVector.get(x);
         double y_k = stateVector.get(y);
         double z_k = stateVector.get(z);

         double xF_k = controlVector.get(xF);
         double yF_k = controlVector.get(yF);

         double zF_k = constants.get(zF);

         double relativeX = x_k - xF_k;
         double relativeY = y_k - yF_k;
         double relativeZ = z_k - zF_k;

         double fX_k = controlVector.get(fx);
         double fY_k = controlVector.get(fy);
         double fZ_k = controlVector.get(fz);

         double tauX_k = controlVector.get(tauX);
         double tauY_k = controlVector.get(tauY);
         double tauZ_k = controlVector.get(tauZ);

         double K_k = controlVector.get(k);

         double currentLength = Math.sqrt(relativeX * relativeX + relativeY * relativeY + relativeZ * relativeZ);

         double nominalPendulumLength = constants.get(nominalLength);
         double normalizedSpringCompression = nominalPendulumLength / currentLength - 1;
         double normalizedSpringForce = K_k * normalizedSpringCompression;

         double l3 = Math.pow(currentLength, -3.0);
         double l5 = Math.pow(currentLength, -5.0);

         double qFx = Q.get(x, x);
         double qFy = Q.get(y, y);
         double qFz = Q.get(z, z);
         double qTx = Q.get(tauX, tauX);
         double qTy = Q.get(tauY, tauY);
         double qTz = Q.get(tauZ, tauZ);

         double Jfxfx = 2.0 * qFx + 2.0 * qTy * relativeZ * relativeZ + 2.0 * qTz * relativeY * relativeY;
         double Jfxfy = -2.0 * qTz * relativeX * relativeY;
         double Jfxfz = -2.0 * qTy * relativeX * relativeZ;
         double Jfxtx = 0.0;
         double Jfxty = -2.0 * qTy * relativeZ;
         double Jfxtz = 2.0 * qTz * relativeY;
         double Jfxxf = 2.0 * qFz * (K_k * (nominalPendulumLength / currentLength - nominalPendulumLength * relativeX * relativeX * l3 - 1.0))
               + 2.0 * qTy * relativeZ * fZ_k  + 2.0 * qTz * relativeY * fY_k;
         double Jfxyf = -2.0 * qFx * (K_k * nominalPendulumLength * relativeX * relativeY * l3)
               + 2.0 * qTz * (-fX_k * relativeY - (tauZ_k + relativeY * fX_k - relativeX * fY_k));
         double Jfxk = -2.0 * qFx * normalizedSpringCompression * relativeX;

         double Jfyfy = 2.0 * qFy + 2.0 * qTx * relativeZ * relativeZ + 2.0 * qTz * relativeX * relativeX;
         double Jfyfz = -2.0 * qTx * relativeZ * relativeY ;
         double Jfytx = 2.0 * qTx * relativeZ;
         double Jfyty = 0.0;
         double Jfytz = -2.0 * qTz * relativeX;
         double Jfyxf = -2.0 * qFy * (K_k * nominalPendulumLength * relativeX * relativeY * l3)
               + 2.0 * qTz * ((-fY_k * relativeX) + (tauZ_k + relativeY * fX_k - relativeX * fY_k));
         double Jfyyf = 2.0 * qFy * (K_k * (nominalPendulumLength / currentLength - nominalPendulumLength * relativeY * relativeY * l3 - 1.0))
               + 2.0 * qTx * relativeZ * fZ_k + 2.0 * qTz * relativeX * fX_k;
         double Jfyk = -2.0 * qFy * (normalizedSpringCompression * relativeY);

         double Jfzfz = 2.0 * qFz + 2.0 * qTx * relativeY * relativeY + 2.0 * qTy * relativeX * relativeX;
         double Jfztx = -2.0 * qTx * relativeY;
         double Jfzty = 2.0 * qTy * relativeX;
         double Jfztz = 0.0;
         double Jfzxf = -2.0 * qFx * (K_k * nominalPendulumLength * relativeX * relativeZ * l3)
               + 2.0 * qTy * (-fZ_k * relativeX - (tauY_k - relativeZ * fX_k + relativeX * fZ_k));
         double Jfzyf = -2.0 * qFx * (K_k * nominalPendulumLength * relativeY * relativeZ * l3)
               + 2.0 * qTx * (-fZ_k * relativeY + (tauX_k + relativeZ * fY_k - relativeY * fZ_k));
         double Jfzk = -2.0 * qFz * (normalizedSpringCompression * relativeZ);

         double Jtxtx = 2.0 * qTx;
         double Jtxty = 0.0;
         double Jtxtz = 0.0;
         double Jtxxf = 0.0;
         double Jtxyf = 2.0 * qTx * fZ_k;
         double Jtxk = 0.0;

         double Jtyty = 2.0 * qTy;
         double Jtytz = 0.0;
         double Jtyxf = -2.0 * qTy * fZ_k;
         double Jtyyf = 0.0;
         double Jtyk = 0.0;

         double Jtztz = 2.0 * qTz;
         double Jtzxf = 2.0 * qTz * fY_k;
         double Jtzyf = -2.0 * qTz * fX_k;
         double Jtzk = 0.0;

         double xForceDifference = fX_k - normalizedSpringForce * relativeX;
         double yForceDifference = fY_k - normalizedSpringForce * relativeY;
         double zForceDifference = fZ_k - normalizedSpringForce * relativeZ;

         double dfXxf = K_k * (-nominalPendulumLength * l3 * relativeX * relativeX + normalizedSpringCompression);
         double dfYxf = -K_k * nominalPendulumLength * l3 * relativeX * relativeY;
         double dfZxf = -K_k * nominalPendulumLength * l3 * relativeX * relativeZ;

         double dfXyf = -K_k * nominalPendulumLength * l3 * relativeX * relativeY;
         double dfYyf = K_k * (-nominalPendulumLength * l3 * relativeY * relativeY + normalizedSpringCompression);
         double dfZyf = -K_k * nominalPendulumLength * l3 * relativeY * relativeZ;

         double dfXxfxf = 3.0 * K_k * nominalPendulumLength * relativeX * l3 - 3.0 * K_k * nominalPendulumLength * Math.pow(relativeX, 3.0) * l5;
         double dfYxfxf = K_k * nominalPendulumLength * relativeY * l3 - 3.0 * K_k * nominalPendulumLength * relativeX * relativeX * relativeY * l5;
         double dfZxfxf = K_k * nominalPendulumLength * relativeZ * l3 - 3.0 * K_k * nominalPendulumLength * relativeX * relativeX * relativeZ * l5;

         double dfXxfyf = K_k * nominalPendulumLength * relativeY * l3 - 3.0 * K_k * nominalPendulumLength * relativeX * relativeX * relativeY * l5;
         double dfYxfyf = K_k * nominalPendulumLength * relativeX * l3 - 3.0 * K_k * nominalPendulumLength * relativeX * relativeY * relativeY * l5;
         double dfZxfyf = -3.0 * K_k * nominalPendulumLength * relativeX * relativeY * relativeZ * l5;

         double dfXyfyf = K_k * nominalPendulumLength * relativeX * l3 - 3.0 * K_k * nominalPendulumLength * relativeX * relativeY * relativeY * l5;
         double dfYyfyf = 3.0 * K_k * nominalPendulumLength * relativeY * l3 - 3.0 * K_k * nominalPendulumLength * Math.pow(relativeY, 3.0) * l5;
         double dfZyfyf = K_k * nominalPendulumLength * relativeZ * l3 - 3.0 * K_k * nominalPendulumLength * relativeY * relativeY * relativeZ * l5;

         double Jxfxf = 2.0 * qFx * (dfXxfxf * xForceDifference + dfXxf * dfXxf);
         Jxfxf += 2.0 * qFy * (dfYxfxf * yForceDifference + dfYxf * dfYxf);
         Jxfxf += 2.0 * qFz * (dfZxfxf * zForceDifference + dfZxf * dfZxf);
         Jxfxf += 2.0 * qTy * fZ_k * fZ_k + 2.0 * qTz * fY_k * fY_k;

         double Jxfyf = 2.0 * qFx * (dfXxfyf * xForceDifference + dfXxf * dfXyf);
         Jxfyf += 2.0 * qFy * (dfYxfyf * yForceDifference + dfYxf * dfYyf);
         Jxfyf += 2.0 * qFz * (dfZxfyf * zForceDifference + dfZxf * dfZyf);
         Jxfyf -= 2.0 * qTz * fY_k * fX_k;

         double Jyfyf = 2.0 * qFx * (dfXyfyf * xForceDifference + dfXyf * dfXyf);
         Jyfyf += 2.0 * qFy * (dfYyfyf * yForceDifference + dfYyf * dfYyf);
         Jyfyf += 2.0 * qFz * (dfZyfyf * zForceDifference + dfZyf * dfZyf);
         Jyfyf += 2.0 * qTx * fZ_k * fZ_k + 2.0 * qTz * fX_k * fX_k;


         double Jxfk = 2.0 * qFx * ((-nominalPendulumLength * l3 * relativeX * relativeX + normalizedSpringCompression) * xForceDifference
               - (normalizedSpringCompression) * relativeX * (K_k * (-nominalPendulumLength * l3 * relativeX * relativeX + normalizedSpringCompression)));
         Jxfk += 2.0 * qFy * ((-nominalPendulumLength * l3 * relativeX * relativeY) * yForceDifference
               - (normalizedSpringCompression) * relativeY * (-K_k * nominalPendulumLength * l3 * relativeX * relativeY));
         Jxfk += 2.0 * qFy * ((-nominalPendulumLength * l3 * relativeX * relativeZ) * zForceDifference
               - (normalizedSpringCompression) * relativeZ * (-K_k * nominalPendulumLength * l3 * relativeX * relativeZ));

         double Jyfk = 2.0 * qFx * ((-nominalPendulumLength * l3 * relativeX * relativeY) * xForceDifference
               - (normalizedSpringCompression) * relativeX * (-K_k * nominalPendulumLength * l3 * relativeX * relativeY));
         Jyfk += 2.0 * qFy * ((-nominalPendulumLength * l3 * relativeY * relativeY + normalizedSpringCompression) * yForceDifference
               - (normalizedSpringCompression) * relativeY * (K_k * (-nominalPendulumLength * l3 * relativeY * relativeY + normalizedSpringCompression)));
         Jyfk += 2.0 * qFz * ((-nominalPendulumLength * l3 * relativeY * relativeZ) * zForceDifference
               - (normalizedSpringCompression) * relativeZ * (-K_k * nominalPendulumLength * l3 * relativeY * relativeZ));


         double Jkk = qFx * relativeX * relativeX + qFy * relativeY * relativeY + qFy * relativeZ * relativeZ;
         Jkk *= 2.0 * normalizedSpringCompression * normalizedSpringCompression;

         matrixToPack.set(fx, fx, Jfxfx);
         matrixToPack.set(fx, fy, Jfxfy);
         matrixToPack.set(fx, fz, Jfxfz);
         matrixToPack.set(fx, tauX, Jfxtx);
         matrixToPack.set(fx, tauY, Jfxty);
         matrixToPack.set(fx, tauZ, Jfxtz);
         matrixToPack.set(fx, xF, Jfxxf);
         matrixToPack.set(fx, yF, Jfxyf);
         matrixToPack.set(fx, k, Jfxk);

         matrixToPack.set(fy, fx, Jfxfy);
         matrixToPack.set(fy, fy, Jfyfy);
         matrixToPack.set(fy, fz, Jfyfz);
         matrixToPack.set(fy, tauX, Jfytx);
         matrixToPack.set(fy, tauY, Jfyty);
         matrixToPack.set(fy, tauZ, Jfytz);
         matrixToPack.set(fy, xF, Jfyxf);
         matrixToPack.set(fy, yF, Jfyyf);
         matrixToPack.set(fy, k, Jfyk);

         matrixToPack.set(fz, fx, Jfxfz);
         matrixToPack.set(fz, fy, Jfyfz);
         matrixToPack.set(fz, fz, Jfzfz);
         matrixToPack.set(fz, tauX, Jfztx);
         matrixToPack.set(fz, tauY, Jfzty);
         matrixToPack.set(fz, tauZ, Jfztz);
         matrixToPack.set(fz, xF, Jfzxf);
         matrixToPack.set(fz, yF, Jfzyf);
         matrixToPack.set(fz, k, Jfzk);

         matrixToPack.set(tauX, fx, Jfxtx);
         matrixToPack.set(tauX, fy, Jfytx);
         matrixToPack.set(tauX, fz, Jfztx);
         matrixToPack.set(tauX, tauX, Jtxtx);
         matrixToPack.set(tauX, tauY, Jtxty);
         matrixToPack.set(tauX, tauZ, Jtxtz);
         matrixToPack.set(tauX, xF, Jtxxf);
         matrixToPack.set(tauX, yF, Jtxyf);
         matrixToPack.set(tauX, k, Jtxk);

         matrixToPack.set(tauY, fx, Jfxty);
         matrixToPack.set(tauY, fy, Jfyty);
         matrixToPack.set(tauY, fz, Jfzty);
         matrixToPack.set(tauY, tauX, Jtxty);
         matrixToPack.set(tauY, tauY, Jtyty);
         matrixToPack.set(tauY, tauZ, Jtytz);
         matrixToPack.set(tauY, xF, Jtyxf);
         matrixToPack.set(tauY, yF, Jtyyf);
         matrixToPack.set(tauY, k, Jtyk);

         matrixToPack.set(tauZ, fx, Jfxtz);
         matrixToPack.set(tauZ, fy, Jfytz);
         matrixToPack.set(tauZ, fz, Jfztz);
         matrixToPack.set(tauZ, tauX, Jtxtz);
         matrixToPack.set(tauZ, tauY, Jtytz);
         matrixToPack.set(tauZ, tauZ, Jtztz);
         matrixToPack.set(tauZ, xF, Jtzxf);
         matrixToPack.set(tauZ, yF, Jtzyf);
         matrixToPack.set(tauZ, k, Jtzk);

         matrixToPack.set(xF, fx, Jfxxf);
         matrixToPack.set(xF, fy, Jfyxf);
         matrixToPack.set(xF, fz, Jfzxf);
         matrixToPack.set(xF, tauX, Jtxxf);
         matrixToPack.set(xF, tauY, Jtyxf);
         matrixToPack.set(xF, tauZ, Jtzxf);
         matrixToPack.set(xF, xF, Jxfxf);
         matrixToPack.set(xF, yF, Jxfyf);
         matrixToPack.set(xF, k, Jxfk);

         matrixToPack.set(yF, fx, Jfxyf);
         matrixToPack.set(yF, fy, Jfyyf);
         matrixToPack.set(yF, fz, Jfzyf);
         matrixToPack.set(yF, tauX, Jtxyf);
         matrixToPack.set(yF, tauY, Jtyyf);
         matrixToPack.set(yF, tauZ, Jtzyf);
         matrixToPack.set(yF, xF, Jxfyf);
         matrixToPack.set(yF, yF, Jyfyf);
         matrixToPack.set(yF, k, Jyfk);

         matrixToPack.set(k, fx, Jfxk);
         matrixToPack.set(k, fy, Jfyk);
         matrixToPack.set(k, fz, Jfzk);
         matrixToPack.set(k, tauX, Jtxk);
         matrixToPack.set(k, tauY, Jtyk);
         matrixToPack.set(k, tauZ, Jtzk);
         matrixToPack.set(k, xF, Jxfk);
         matrixToPack.set(k, yF, Jyfk);
         matrixToPack.set(k, k, Jkk);
         break;
      case FLIGHT:

         break;
      }
   }

   @Override
   public void getCostStateGradientOfControlGradient(SLIPState hybridState, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F constants,
                                                     DenseMatrix64F matrixToPack)
   {

   }

   @Override
   public void getCostControlGradientOfStateGradient(SLIPState hybridState, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F constants,
                                                     DenseMatrix64F matrixToPack)
   {
      if (matrixToPack.getNumRows() != stateVectorSize)
         throw new RuntimeException("The hessian has the wrong number of rows.");
      if (matrixToPack.getNumCols() != controlVectorSize)
         throw new RuntimeException("The hessian has the wrong number of cols.");

      matrixToPack.zero();

      switch (hybridState)
      {
      case STANCE:
         double x_k = stateVector.get(x);
         double y_k = stateVector.get(y);
         double z_k = stateVector.get(z);

         double xF_k = controlVector.get(xF);
         double yF_k = controlVector.get(yF);

         double zF_k = constants.get(zF);

         double relativeX = x_k - xF_k;
         double relativeY = y_k - yF_k;
         double relativeZ = z_k - zF_k;

         double fX_k = controlVector.get(fx);
         double fY_k = controlVector.get(fy);
         double fZ_k = controlVector.get(fz);

         double tauX_k = controlVector.get(tauX);
         double tauY_k = controlVector.get(tauY);
         double tauZ_k = controlVector.get(tauZ);

         double K_k = controlVector.get(k);

         double currentLength = Math.sqrt(relativeX * relativeX + relativeY * relativeY + relativeZ * relativeZ);

         double nominalPendulumLength = constants.get(nominalLength);
         double normalizedSpringCompression = nominalPendulumLength / currentLength - 1;
         double normalizedSpringForce = K_k * normalizedSpringCompression;

         double l3 = Math.pow(currentLength, -3.0);
         double l5 = Math.pow(currentLength, -5.0);

         double qFx = Q.get(x, x);
         double qFy = Q.get(y, y);
         double qFz = Q.get(z, z);
         double qTx = Q.get(tauX, tauX);
         double qTy = Q.get(tauY, tauY);
         double qTz = Q.get(tauZ, tauZ);

         double xForceDifference = fX_k - normalizedSpringForce * relativeX;
         double yForceDifference = fY_k - normalizedSpringForce * relativeY;
         double zForceDifference = fZ_k - normalizedSpringForce * relativeZ;

         double dfXdx = K_k * (nominalPendulumLength * l3 * relativeX * relativeX - normalizedSpringCompression);
         double dfYdx = K_k * nominalPendulumLength * l3 * relativeX * relativeY;
         double dfZdx = K_k * nominalPendulumLength * l3 * relativeX * relativeZ;

         double dfXdy = K_k * nominalPendulumLength * l3 * relativeX * relativeY;
         double dfYdy = K_k * (nominalPendulumLength * l3 * relativeY * relativeY - normalizedSpringCompression);
         double dfZdy = K_k * nominalPendulumLength * l3 * relativeY * relativeZ;

         double dfXdz = K_k * nominalPendulumLength * l3 * relativeX * relativeZ;
         double dfYdz = K_k * nominalPendulumLength * l3 * relativeY * relativeZ;
         double dfZdz = K_k * (nominalPendulumLength * l3 * relativeZ * relativeZ - normalizedSpringCompression);

         double dfXdxdxf = 3.0 * K_k * nominalPendulumLength * Math.pow(relativeX, 3.0) * l5 - 3.0 * K_k * nominalPendulumLength * relativeX * l3;
         double dfYdxdxf = 3.0 * K_k * nominalPendulumLength * relativeX * relativeX * relativeY * l5 - K_k * nominalPendulumLength * relativeY * l3;
         double dfZdxdxf = 3.0 * K_k * nominalPendulumLength * relativeX * relativeX * relativeZ * l5 - K_k * nominalPendulumLength * relativeZ * l3;

         double dfXdxdyf = 3.0 * K_k * nominalPendulumLength * relativeX * relativeX * relativeY * l5 - K_k * nominalPendulumLength * relativeY * l3;
         double dfYdxdyf = 3.0 * K_k * nominalPendulumLength * relativeX * relativeY * relativeY * l5 - K_k * nominalPendulumLength * relativeX * l3;
         double dfZdxdyf = 3.0 * K_k * nominalPendulumLength * relativeX * relativeY * relativeZ * l5;

         double Jxfx = 2.0 * qFx * (K_k * (nominalPendulumLength * l3 * relativeX * relativeX - normalizedSpringCompression))
               - 2.0 * qTy * fZ_k * relativeZ - 2.0 * qTz * fY_k * relativeY;
         double Jxfy = 2.0 * qFy * K_k * nominalPendulumLength * l3 * relativeX * relativeY
               + 2.0 * qTz * (fY_k * relativeX - (tauZ_k + relativeY * fX_k - relativeX * fY_k));
         double Jxfz = 2.0 * qFz * K_k * nominalPendulumLength * l3 * relativeX * relativeZ
               + 2.0 * qTy * (fZ_k * relativeX + (tauY_k - relativeZ * fX_k + relativeX * fZ_k));

         double Jxtaux = 0.0;
         double Jxtauy = 2.0 * qTy * fZ_k;
         double Jxtauz = -2.0 * qTz * fY_k;

         double Jxxf = 2.0 * qFx * (dfXdxdxf * xForceDifference - dfXdx * dfXdx);
         Jxxf += 2.0 * qFy * (dfYdxdxf * yForceDifference - dfYdx * dfYdx);
         Jxxf += 2.0 * qFz * (dfZdxdxf * zForceDifference - dfZdx * dfZdx);
         Jxxf -= (2.0 * qTy * fZ_k * fZ_k + 2.0 * qTz * fY_k * fY_k);

         double Jxyf = 2.0 * qFx * (dfXdxdyf * xForceDifference - dfXdx * dfXdy);
         Jxyf += 2.0 * qFy * (dfYdxdyf * yForceDifference - dfYdx * dfYdy);
         Jxyf += 2.0 * qFz * (dfZdxdyf * zForceDifference - dfZdx * dfZdy);
         Jxyf += 2.0 * qTz  * fX_k * fY_k;

         double Jxk = 2.0 * qFx * ((nominalPendulumLength * l3 * relativeX * relativeX - normalizedSpringCompression) * xForceDifference - dfXdx * normalizedSpringCompression * relativeX);
         Jxk += 2.0 * qFy * ((nominalPendulumLength * l3 * relativeX * relativeY) * yForceDifference - dfYdx * normalizedSpringCompression * relativeY);
         Jxk += 2.0 * qFz * ((nominalPendulumLength * l3 * relativeX * relativeZ) * zForceDifference - dfZdx * normalizedSpringCompression * relativeZ);



         double Jyfx = 2.0 * qFx * (K_k * nominalPendulumLength * l3 * relativeX * relativeY)
               + 2.0 * qTz * (fX_k * relativeY + (tauZ_k + relativeY * fX_k - relativeX * fY_k));
         double Jyfy = 2.0 * qFy * (K_k * (nominalPendulumLength * l3 * relativeY * relativeY - normalizedSpringCompression))
               - 2.0 * qTx * fZ_k * relativeZ - 2.0 * qTz * fX_k * relativeX;
         double Jyfz = 2.0 * qFz * K_k * nominalPendulumLength * l3 * relativeY * relativeZ
               + 2.0 * qTx * (fZ_k * relativeY - (tauX_k + relativeZ * fY_k - relativeY * fZ_k));

         double Jytaux = -2.0 * qTx * fZ_k;
         double Jytauy = 0.0;
         double Jytauz = 2.0 * qTz * fX_k;


         double dfXdydxf = 3.0 * K_k * nominalPendulumLength * relativeX * relativeX * relativeY * l5 - K_k * nominalPendulumLength * relativeY * l3;
         double dfYdydxf = 3.0 * K_k * nominalPendulumLength * relativeX * relativeY * relativeY * l5 - K_k * nominalPendulumLength * relativeX * l3;
         double dfZdydxf = 3.0 * K_k * nominalPendulumLength * relativeX * relativeY * relativeZ * l5;

         double dfXdydyf = 3.0 * K_k * nominalPendulumLength * relativeX * relativeY * relativeY * l5 - K_k * nominalPendulumLength * relativeX * l3;
         double dfYdydyf = 3.0 * K_k * nominalPendulumLength * Math.pow(relativeY, 3.0) * l5 - 3.0 * K_k * nominalPendulumLength * relativeY * l3;
         double dfZdydyf = 3.0 * K_k * nominalPendulumLength * relativeY * relativeY * relativeZ * l5 - K_k * nominalPendulumLength * relativeZ * l3;

         double Jyxf = 2.0 * qFx * (dfXdydxf * xForceDifference - dfXdx * dfXdy);
         Jyxf += 2.0 * qFy * (dfYdydxf * yForceDifference - dfYdx * dfYdy);
         Jyxf += 2.0 * qFz * (dfZdydxf * zForceDifference - dfZdx * dfZdy);
         Jyxf += 2.0 * qTz * fX_k * fY_k;

         double Jyyf = 2.0 * qFx * (dfXdydyf * xForceDifference - dfXdy * dfXdy);
         Jyyf += 2.0 * qFy * (dfYdydyf * yForceDifference - dfYdy * dfYdy);
         Jyyf += 2.0 * qFz * (dfZdydyf * zForceDifference - dfZdy * dfZdy);
         Jyyf -= 2.0 * (qTx * fZ_k * fZ_k + qTz * fX_k * fX_k);

         double Jyk = 2.0 * qFx * ((nominalPendulumLength * l3 * relativeX * relativeY) * xForceDifference - normalizedSpringCompression * relativeX * dfXdy);
         Jyk += 2.0 * qFy * ((nominalPendulumLength * l3 * relativeY * relativeY - normalizedSpringCompression) * yForceDifference - normalizedSpringCompression * relativeY * dfYdy);
         Jyk += 2.0 * qFz * ((nominalPendulumLength * l3 * relativeY * relativeZ) * zForceDifference - normalizedSpringCompression * relativeZ * dfZdy);

         double Jzfx = 2.0 * qFx * (K_k * nominalPendulumLength * l3 * relativeX * relativeZ)
               + 2.0 * qTy * (fX_k * relativeZ - (tauY_k - relativeZ * fX_k + relativeX * fZ_k));
         double Jzfy = 2.0 * qFy * (K_k * nominalPendulumLength * l3 * relativeY * relativeZ)
               + 2.0 * qTx * (fY_k * relativeZ + (tauX_k + relativeZ * fY_k - relativeY * fZ_k));
         double Jzfz = 2.0 * qFz * (K_k * (nominalPendulumLength * l3 * relativeZ * relativeZ - normalizedSpringCompression))
               - 2.0 * qTx * fY_k * relativeY - 2.0 * qTy * fX_k * relativeX;

         double Jztaux = 2.0 * qTx * fY_k;
         double Jztauy = -2.0 * qTy * fX_k;
         double Jztauz = 0.0;

         double dfXdzdxf = 3.0 * K_k * nominalPendulumLength * relativeX * relativeX * relativeZ * l5 - K_k * nominalPendulumLength * relativeZ * l3;
         double dfYdzdxf = 3.0 * K_k * nominalPendulumLength * relativeX * relativeY * relativeZ * l5;
         double dfZdzdxf = 3.0 * K_k * nominalPendulumLength * relativeX * relativeZ * relativeZ * l5 - K_k * nominalPendulumLength * relativeX * l3;

         double dfXdzdyf = 3.0 * K_k * nominalPendulumLength * relativeX * relativeY * relativeZ * l5;
         double dfYdzdyf = 3.0 * K_k * nominalPendulumLength * relativeY * relativeY * relativeZ * l5 - K_k * nominalPendulumLength * relativeZ * l3;
         double dfZdzdyf = 3.0 * K_k * nominalPendulumLength * relativeZ * relativeZ * relativeY * l5 - K_k * nominalPendulumLength * relativeY * l3;

         double Jzxf = 2.0 * qFx * (dfXdzdxf * xForceDifference - dfXdx * dfXdz);
         Jzxf += 2.0 * qFy * (dfYdzdxf * yForceDifference - dfYdx * dfYdz);
         Jzxf += 2.0 * qFz * (dfZdzdxf * zForceDifference - dfZdx * dfZdz);
         Jzxf += 2.0 * qTz * fX_k * fZ_k;

         double Jzyf = 2.0 * qFx * (dfXdzdyf * xForceDifference - dfXdz * dfXdy);
         Jzyf += 2.0 * qFy * (dfYdzdyf * yForceDifference - dfYdz * dfYdy);
         Jzyf += 2.0 * qFz * (dfZdzdyf * zForceDifference - dfZdz * dfZdy);
         Jzyf += 2.0 * qTx * fY_k * fZ_k;

         double Jzk = 2.0 * qFx * (nominalPendulumLength * l3 * relativeX * relativeZ * xForceDifference - normalizedSpringCompression * relativeX * dfXdz);
         Jzk += 2.0 * qFy * (nominalPendulumLength * l3 * relativeY * relativeZ * yForceDifference - normalizedSpringCompression * relativeY * dfYdz);
         Jzk += 2.0 * qFz * ((nominalPendulumLength * l3 * relativeZ * relativeZ - normalizedSpringCompression) * zForceDifference - normalizedSpringCompression * relativeZ * dfZdz);

         matrixToPack.set(x, fx, Jxfx);
         matrixToPack.set(x, fy, Jxfy);
         matrixToPack.set(x, fz, Jxfz);
         matrixToPack.set(x, tauX, Jxtaux);
         matrixToPack.set(x, tauY, Jxtauy);
         matrixToPack.set(x, tauZ, Jxtauz);
         matrixToPack.set(x, xF, Jxxf);
         matrixToPack.set(x, yF, Jxyf);
         matrixToPack.set(x, k, Jxk);

         matrixToPack.set(y, fx, Jyfx);
         matrixToPack.set(y, fy, Jyfy);
         matrixToPack.set(y, fz, Jyfz);
         matrixToPack.set(y, tauX, Jytaux);
         matrixToPack.set(y, tauY, Jytauy);
         matrixToPack.set(y, tauZ, Jytauz);
         matrixToPack.set(y, xF, Jyxf);
         matrixToPack.set(y, yF, Jyyf);
         matrixToPack.set(y, k, Jyk);

         matrixToPack.set(z, fx, Jzfx);
         matrixToPack.set(z, fy, Jzfy);
         matrixToPack.set(z, fz, Jzfz);
         matrixToPack.set(z, tauX, Jztaux);
         matrixToPack.set(z, tauY, Jztauy);
         matrixToPack.set(z, tauZ, Jztauz);
         matrixToPack.set(z, xF, Jzxf);
         matrixToPack.set(z, yF, Jzyf);
         matrixToPack.set(z, k, Jzk);

         break;
      case FLIGHT:
         break;
      }

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
