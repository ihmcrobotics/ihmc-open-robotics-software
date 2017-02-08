package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer.TransferInitialICPVelocityMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation.CubicMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class InitialICPVelocityCurrentMultiplier
{
   private final CubicMatrix cubicMatrix;
   private final CubicDerivativeMatrix cubicDerivativeMatrix;

   private final TransferInitialICPVelocityMatrix transferInitialICPVelocityMatrix;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable positionMultiplier;
   private final DoubleYoVariable velocityMultiplier;

   public InitialICPVelocityCurrentMultiplier(YoVariableRegistry registry)
   {
      positionMultiplier = new DoubleYoVariable("InitialICPVelocityCurrentMultiplier", registry);
      velocityMultiplier = new DoubleYoVariable("InitialICPCVelocityCurrentVelocityMultiplier", registry);

      cubicMatrix = new CubicMatrix();
      cubicDerivativeMatrix = new CubicDerivativeMatrix();

      transferInitialICPVelocityMatrix = new TransferInitialICPVelocityMatrix();
   }

   public void reset()
   {
      positionMultiplier.set(0.0);
      velocityMultiplier.set(0.0);
   }

   public double getPositionMultiplier()
   {
      return positionMultiplier.getDoubleValue();
   }

   public double getVelocityMultiplier()
   {
      return velocityMultiplier.getDoubleValue();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, double timeInState, boolean isInTransfer)
   {
      double positionMultiplier, velocityMultiplier;
      if (isInTransfer)
         positionMultiplier = computeInTransfer(doubleSupportDurations, timeInState);
      else
         positionMultiplier = 0.0;
      this.positionMultiplier.set(positionMultiplier);

      if (isInTransfer)
         velocityMultiplier = computeInTransferVelocity();
      else
         velocityMultiplier = 0.0;

      this.velocityMultiplier.set(velocityMultiplier);
   }

   private double computeInTransfer(ArrayList<DoubleYoVariable> doubleSupportDurations, double timeInState)
   {
      transferInitialICPVelocityMatrix.compute();

      double splineDuration = doubleSupportDurations.get(0).getDoubleValue();

      cubicDerivativeMatrix.setSegmentDuration(splineDuration);
      cubicDerivativeMatrix.update(timeInState, true);
      cubicMatrix.setSegmentDuration(splineDuration);
      cubicMatrix.update(timeInState, true);

      CommonOps.mult(cubicMatrix, transferInitialICPVelocityMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeInTransferVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, transferInitialICPVelocityMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }
}
