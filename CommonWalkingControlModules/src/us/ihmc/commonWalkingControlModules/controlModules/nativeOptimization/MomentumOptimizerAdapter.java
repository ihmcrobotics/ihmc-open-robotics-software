package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.wrenchDistribution.CylinderAndPlaneContactMatrixCalculatorAdapter;
import us.ihmc.utilities.exeptions.NoConvergenceException;

public class MomentumOptimizerAdapter
{
   private CVXMomentumOptimizerWithGRFPenalizedSmootherNative momentumOptimizerWithGRFPenalizedSmootherNative;
   private CVXMomentumOptimizerWithGRFPenalizedSmootherNativeInput momentumOptimizerWithGRFPenalizedSmootherNativeInput;

   private final int rhoSize;

   private final DenseMatrix64F rhoPrevious;
   private final DenseMatrix64F wRhoSmoother;
   private final DenseMatrix64F rhoPreviousMean;
   private final DenseMatrix64F wRhoPenalizer;

   private DenseMatrix64F outputRho, outputPhi, outputJointAccelerations;
   private double outputOptVal;

   public MomentumOptimizerAdapter(int nDoF)
   {
      rhoSize = CVXMomentumOptimizerWithGRFPenalizedSmootherNative.rhoSize;

      momentumOptimizerWithGRFPenalizedSmootherNative = new CVXMomentumOptimizerWithGRFPenalizedSmootherNative(nDoF, rhoSize);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput = new CVXMomentumOptimizerWithGRFPenalizedSmootherNativeInput();

      outputRho = new DenseMatrix64F(rhoSize, 1);
      rhoPrevious = new DenseMatrix64F(rhoSize, 1);
      wRhoSmoother = new DenseMatrix64F(rhoSize, rhoSize);
      rhoPreviousMean = new DenseMatrix64F(rhoSize, 1);
      wRhoPenalizer = new DenseMatrix64F(rhoSize, rhoSize);
   }

   public void reset()
   {
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.reset();
   }

   public int getRhoSize()
   {
      return rhoSize;
   }

   public void setInputs(DenseMatrix64F a, DenseMatrix64F b, CylinderAndPlaneContactMatrixCalculatorAdapter wrenchMatrixCalculator,
                         DenseMatrix64F wrenchEquationRightHandSide, DenseMatrix64F momentumDotWeight, DenseMatrix64F dampedLeastSquaresFactorMatrix,
                         DenseMatrix64F jSecondary, DenseMatrix64F pSecondary, DenseMatrix64F weightMatrixSecondary)
   {
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setCentroidalMomentumMatrix(a);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setMomentumDotEquationRightHandSide(b);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setContactPointWrenchMatrix(wrenchMatrixCalculator.getQRho());
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setRhoMin(wrenchMatrixCalculator.getRhoMin());
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setRhoPrevious(rhoPrevious);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setWrenchEquationRightHandSide(wrenchEquationRightHandSide);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setMomentumDotWeight(momentumDotWeight);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setJointAccelerationRegularization(dampedLeastSquaresFactorMatrix);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setSecondaryConstraintJacobian(jSecondary);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setSecondaryConstraintRightHandSide(pSecondary);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setSecondaryConstraintWeight(weightMatrixSecondary);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setGroundReactionForceRegularization(wrenchMatrixCalculator.getWRho());
      wrenchMatrixCalculator.packWRhoSmoother(wRhoSmoother);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setRateOfChangeOfGroundReactionForceRegularization(wRhoSmoother);

      wrenchMatrixCalculator.packRhoPreviousAverageForEndEffectors(rhoPreviousMean);
      wrenchMatrixCalculator.packWRhoPenalizer(wRhoPenalizer);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setRhoPreviousAverage(rhoPreviousMean);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setCenterOfPressurePenalizedRegularization(wRhoPenalizer);
   }

   public void solve() throws NoConvergenceException
   {
      rhoPrevious.set(outputRho);

      try
      {
         momentumOptimizerWithGRFPenalizedSmootherNative.solve(momentumOptimizerWithGRFPenalizedSmootherNativeInput);
      }
      finally
      {
         CVXMomentumOptimizerWithGRFPenalizedSmootherNativeOutput momentumOptimizerWithGRFPenalizedSmootherNativeOutput =
               momentumOptimizerWithGRFPenalizedSmootherNative.getOutput();
         outputRho = momentumOptimizerWithGRFPenalizedSmootherNativeOutput.getRho();
         outputJointAccelerations = momentumOptimizerWithGRFPenalizedSmootherNativeOutput.getJointAccelerations();
         outputOptVal = momentumOptimizerWithGRFPenalizedSmootherNativeOutput.getOptVal();
      }
   }

   public DenseMatrix64F getOutputRho()
   {
      return outputRho;
   }

   public DenseMatrix64F getOutputPhi()
   {
      return outputPhi;
   }

   public DenseMatrix64F getOutputJointAccelerations()
   {
      return outputJointAccelerations;
   }

   public double getOutputOptVal()
   {
      return outputOptVal;
   }
}
