package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.current;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.NewSwingInitialICPMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer.NewTransferInitialICPMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer.NewTransferInitialICPVelocityMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicDerivativeMatrix;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.interpolation.CubicMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class NewInitialICPVelocityCurrentMultiplier
{
   private final CubicMatrix cubicMatrix;
   private final CubicDerivativeMatrix cubicDerivativeMatrix;

   private final NewTransferInitialICPVelocityMatrix transferInitialICPVelocityMatrix;

   private final DenseMatrix64F matrixOut = new DenseMatrix64F(1, 1);

   private final DoubleYoVariable positionMultiplier;
   private final DoubleYoVariable velocityMultiplier;

   public NewInitialICPVelocityCurrentMultiplier(YoVariableRegistry registry)
   {
      positionMultiplier = new DoubleYoVariable("InitialICPVelocityCurrentMultiplier", registry);
      velocityMultiplier = new DoubleYoVariable("InitialICPCVelocityCurrentVelocityMultiplier", registry);

      cubicMatrix = new CubicMatrix();
      cubicDerivativeMatrix = new CubicDerivativeMatrix();

      transferInitialICPVelocityMatrix = new NewTransferInitialICPVelocityMatrix();
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

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, double timeRemaining, boolean isInTransfer)
   {
      double positionMultiplier, velocityMultiplier;
      if (isInTransfer)
         positionMultiplier = computeInTransfer(doubleSupportDurations, timeRemaining);
      else
         positionMultiplier = 0.0;
      this.positionMultiplier.set(positionMultiplier);

      if (isInTransfer)
         velocityMultiplier = computeInTransferVelocity();
      else
         velocityMultiplier = 0.0;

      this.velocityMultiplier.set(velocityMultiplier);
   }

   private double computeInTransfer(ArrayList<DoubleYoVariable> doubleSupportDurations, double timeRemaining)
   {
      transferInitialICPVelocityMatrix.compute();

      double splineDuration = doubleSupportDurations.get(0).getDoubleValue();

      cubicDerivativeMatrix.setSegmentDuration(splineDuration);
      cubicDerivativeMatrix.update(timeRemaining);
      cubicMatrix.setSegmentDuration(splineDuration);
      cubicMatrix.update(timeRemaining);

      CommonOps.mult(cubicMatrix, transferInitialICPVelocityMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }

   private double computeInTransferVelocity()
   {
      CommonOps.mult(cubicDerivativeMatrix, transferInitialICPVelocityMatrix, matrixOut);

      return matrixOut.get(0, 0);
   }
}
