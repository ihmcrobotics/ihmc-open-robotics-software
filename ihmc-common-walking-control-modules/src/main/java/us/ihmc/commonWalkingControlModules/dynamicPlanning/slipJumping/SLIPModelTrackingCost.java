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

   public SLIPModelTrackingCost(double pendulumMass, double nominalPendulumLength, double gravityZ)
   {
      this.pendulumMass = pendulumMass;
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
