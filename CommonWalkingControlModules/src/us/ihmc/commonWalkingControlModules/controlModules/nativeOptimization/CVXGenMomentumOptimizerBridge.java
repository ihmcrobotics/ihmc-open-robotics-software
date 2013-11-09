package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylinderAndPlaneContactMatrixCalculatorAdapter;
import us.ihmc.utilities.exeptions.NoConvergenceException;

public class CVXGenMomentumOptimizerBridge
{
   private final CVXWithCylinderNative momentumOptimizerNative;
   private final CVXWithCylinderNativeInput momentumOptimizerNativeInput;

   private final CVXMomentumOptimizerWithGRFSmootherNative momentumOptimizerWithGRFSmootherNative;
   private final CVXMomentumOptimizerWithGRFSmootherNativeInput momentumOptimizerWithGRFSmootherNativeInput;

   private final int rhoSize;
   private final int phiSize;
   
   private final DenseMatrix64F rhoPrevious;
   private final DenseMatrix64F wRhoSmoother;

   private DenseMatrix64F outputRho, outputPhi, outputJointAccelerations;
   private double outputOptVal;
   
   public enum MomentumOptimizer {NO_GRF_SMOOTHER, GRF_SMOOTHER};
   private MomentumOptimizer activeMomentumOptimizer;
   
   public CVXGenMomentumOptimizerBridge(int nDoF, MomentumOptimizer momentumOptimizerToUse, MomentumOptimizationSettings momentumOptimizationSettings)
   {
      this.activeMomentumOptimizer = momentumOptimizerToUse;
      
      int cvxWithCylinderNativeRhoSize = CVXWithCylinderNative.rhoSize;
      int cvxMomentumOptimizerWithGRFSmootherNativeRhoSize = CVXMomentumOptimizerWithGRFSmootherNative.rhoSize;
      rhoSize = Math.min(cvxWithCylinderNativeRhoSize, cvxMomentumOptimizerWithGRFSmootherNativeRhoSize);

      if (cvxWithCylinderNativeRhoSize != cvxMomentumOptimizerWithGRFSmootherNativeRhoSize)
      {
         System.err.println("CVXWithCylinderNative.rhoSize = " + cvxWithCylinderNativeRhoSize + " and CVXMomentumOptimizerWithGRFSmootherNative.rhoSize = " + cvxMomentumOptimizerWithGRFSmootherNativeRhoSize);
         System.err.println("Setup the momentum optimizer bridge for rhoSize = " + rhoSize);
      }  
      
      int cvxWithCylinderNativePhiSize = CVXWithCylinderNative.phiSize;
      int cvxMomentumOptimizerWithGRFSmootherNativePhiSize = CVXMomentumOptimizerWithGRFSmootherNative.phiSize;
      phiSize = Math.min(cvxWithCylinderNativePhiSize, cvxMomentumOptimizerWithGRFSmootherNativePhiSize);

      if (cvxWithCylinderNativePhiSize != cvxMomentumOptimizerWithGRFSmootherNativePhiSize)
      {
         System.err.println("CVXWithCylinderNative.phiSize = " + cvxWithCylinderNativePhiSize + " and CVXMomentumOptimizerWithGRFSmootherNative.phiSize = " + cvxMomentumOptimizerWithGRFSmootherNativePhiSize);
         System.err.println("Setup the momentum optimizer bridge for phiSize = " + phiSize);
      }  

      momentumOptimizerNative = new CVXWithCylinderNative(nDoF, rhoSize, phiSize);
      momentumOptimizerNativeInput = new CVXWithCylinderNativeInput();

      momentumOptimizerWithGRFSmootherNative = new CVXMomentumOptimizerWithGRFSmootherNative(nDoF, rhoSize, phiSize);
      momentumOptimizerWithGRFSmootherNativeInput = new CVXMomentumOptimizerWithGRFSmootherNativeInput();
      
      outputRho = new DenseMatrix64F(rhoSize, 1);
      rhoPrevious = new DenseMatrix64F(rhoSize, 1);
      wRhoSmoother = new DenseMatrix64F(rhoSize, rhoSize);
   }
   
   public void setActiveMomentumOptimizer(MomentumOptimizer activeMomentumOptimizer)
   {
      this.activeMomentumOptimizer = activeMomentumOptimizer;
   }

   public void setRateOfChangeOfGroundReactionForceRegularization(DenseMatrix64F wRhoSmoother)
   {
      momentumOptimizerWithGRFSmootherNativeInput.setRateOfChangeOfGroundReactionForceRegularization(wRhoSmoother);
   }

   public void reset()
   {
      switch(activeMomentumOptimizer)
      {
      case NO_GRF_SMOOTHER:
         momentumOptimizerNativeInput.reset();
         break;
      case GRF_SMOOTHER:
         momentumOptimizerWithGRFSmootherNativeInput.reset();
         break;
      default:
         throw new RuntimeException("Should not get there");
      }
   }

   public void setInputs(DenseMatrix64F a, DenseMatrix64F b, CylinderAndPlaneContactMatrixCalculatorAdapter wrenchMatrixCalculator,
         DenseMatrix64F wrenchEquationRightHandSide, DenseMatrix64F momentumDotWeight, DenseMatrix64F dampedLeastSquaresFactorMatrix,
         DenseMatrix64F jSecondary, DenseMatrix64F pSecondary, DenseMatrix64F weightMatrixSecondary)
   {
      switch(activeMomentumOptimizer)
      {
      case NO_GRF_SMOOTHER:
         momentumOptimizerNativeInput.setCentroidalMomentumMatrix(a);
         momentumOptimizerNativeInput.setMomentumDotEquationRightHandSide(b);
         momentumOptimizerNativeInput.setContactPointWrenchMatrix(wrenchMatrixCalculator.getQRho());
         momentumOptimizerNativeInput.setContactPointWrenchMatrixForBoundedCylinderVariables(wrenchMatrixCalculator.getQPhi());
         momentumOptimizerNativeInput.setPhiMin(wrenchMatrixCalculator.getPhiMin());
         momentumOptimizerNativeInput.setPhiMax(wrenchMatrixCalculator.getPhiMax());
         momentumOptimizerNativeInput.setRhoMin(wrenchMatrixCalculator.getRhoMin());
         momentumOptimizerNativeInput.setWrenchEquationRightHandSide(wrenchEquationRightHandSide);
         momentumOptimizerNativeInput.setMomentumDotWeight(momentumDotWeight);
         momentumOptimizerNativeInput.setJointAccelerationRegularization(dampedLeastSquaresFactorMatrix);
         momentumOptimizerNativeInput.setSecondaryConstraintJacobian(jSecondary);
         momentumOptimizerNativeInput.setSecondaryConstraintRightHandSide(pSecondary);
         momentumOptimizerNativeInput.setSecondaryConstraintWeight(weightMatrixSecondary);
         momentumOptimizerNativeInput.setGroundReactionForceRegularization(wrenchMatrixCalculator.getWRho());
         momentumOptimizerNativeInput.setPhiRegularization(wrenchMatrixCalculator.getWPhi());
         
         break;
      case GRF_SMOOTHER:
         momentumOptimizerWithGRFSmootherNativeInput.setCentroidalMomentumMatrix(a);
         momentumOptimizerWithGRFSmootherNativeInput.setMomentumDotEquationRightHandSide(b);
         momentumOptimizerWithGRFSmootherNativeInput.setContactPointWrenchMatrix(wrenchMatrixCalculator.getQRho());
         momentumOptimizerWithGRFSmootherNativeInput.setContactPointWrenchMatrixForBoundedCylinderVariables(wrenchMatrixCalculator.getQPhi());
         momentumOptimizerWithGRFSmootherNativeInput.setPhiMin(wrenchMatrixCalculator.getPhiMin());
         momentumOptimizerWithGRFSmootherNativeInput.setPhiMax(wrenchMatrixCalculator.getPhiMax());
         momentumOptimizerWithGRFSmootherNativeInput.setRhoMin(wrenchMatrixCalculator.getRhoMin());
         momentumOptimizerWithGRFSmootherNativeInput.setRhoPrevious(rhoPrevious);
         momentumOptimizerWithGRFSmootherNativeInput.setWrenchEquationRightHandSide(wrenchEquationRightHandSide);
         momentumOptimizerWithGRFSmootherNativeInput.setMomentumDotWeight(momentumDotWeight);
         momentumOptimizerWithGRFSmootherNativeInput.setJointAccelerationRegularization(dampedLeastSquaresFactorMatrix);
         momentumOptimizerWithGRFSmootherNativeInput.setSecondaryConstraintJacobian(jSecondary);
         momentumOptimizerWithGRFSmootherNativeInput.setSecondaryConstraintRightHandSide(pSecondary);
         momentumOptimizerWithGRFSmootherNativeInput.setSecondaryConstraintWeight(weightMatrixSecondary);
         momentumOptimizerWithGRFSmootherNativeInput.setGroundReactionForceRegularization(wrenchMatrixCalculator.getWRho());
         momentumOptimizerWithGRFSmootherNativeInput.setPhiRegularization(wrenchMatrixCalculator.getWPhi());
         wrenchMatrixCalculator.packWRhoSmoother(wRhoSmoother);
         momentumOptimizerWithGRFSmootherNativeInput.setRateOfChangeOfGroundReactionForceRegularization(wRhoSmoother);
         break;
      default:
         throw new RuntimeException("Should not get there");
      }
   }

   public void solve() throws NoConvergenceException
   {
      switch(activeMomentumOptimizer)
      {
      case NO_GRF_SMOOTHER:
         momentumOptimizerNative.solve(momentumOptimizerNativeInput);
         CVXWithCylinderNativeOutput momentumOptimizerNativeOutput = momentumOptimizerNative.getOutput();
         outputRho = momentumOptimizerNativeOutput.getRho();
         outputPhi = momentumOptimizerNativeOutput.getPhi();
         outputJointAccelerations = momentumOptimizerNativeOutput.getJointAccelerations();
         outputOptVal = momentumOptimizerNativeOutput.getOptVal();
         break;
      case GRF_SMOOTHER:
         rhoPrevious.set(outputRho);
         momentumOptimizerWithGRFSmootherNative.solve(momentumOptimizerWithGRFSmootherNativeInput);
         CVXMomentumOptimizerWithGRFSmootherNativeOutput momentumOptimizerWithGRFSmootherNativeOutput = momentumOptimizerWithGRFSmootherNative.getOutput();
         outputRho = momentumOptimizerWithGRFSmootherNativeOutput.getRho();
         outputPhi = momentumOptimizerWithGRFSmootherNativeOutput.getPhi();
         outputJointAccelerations = momentumOptimizerWithGRFSmootherNativeOutput.getJointAccelerations();
         outputOptVal = momentumOptimizerWithGRFSmootherNativeOutput.getOptVal();
         break;
      default:
         throw new RuntimeException("Should not get there");
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
