package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.utilities.exceptions.NoConvergenceException;
import us.ihmc.yoUtilities.math.frames.YoMatrix;

public class CVXMomentumOptimizerAdapter implements MomentumOptimizerInterface
{
   private CVXMomentumOptimizerWithGRFPenalizedSmootherNative momentumOptimizerWithGRFPenalizedSmootherNative;
   private CVXMomentumOptimizerWithGRFPenalizedSmootherNativeInput momentumOptimizerWithGRFPenalizedSmootherNativeInput;

   private final int rhoSize;
   private final int nSupportVectors;
   private final int nPointsPerPlane;
   private final int nPlanes;

   private final DenseMatrix64F rhoPrevious;
   private final YoMatrix rhoPreviousYoMatrix;
   
   private DenseMatrix64F outputRho, outputJointAccelerations;
   private double outputOptVal;

   public CVXMomentumOptimizerAdapter(int nDoF, YoVariableRegistry registry)
   {
      rhoSize = CVXMomentumOptimizerWithGRFPenalizedSmootherNative.rhoSize;
      nSupportVectors = CVXMomentumOptimizerWithGRFPenalizedSmootherNative.nSupportVectors;
      nPointsPerPlane = CVXMomentumOptimizerWithGRFPenalizedSmootherNative.nPointsPerPlane;
      nPlanes = CVXMomentumOptimizerWithGRFPenalizedSmootherNative.nPlanes;

      momentumOptimizerWithGRFPenalizedSmootherNative = new CVXMomentumOptimizerWithGRFPenalizedSmootherNative(nDoF, rhoSize);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput = new CVXMomentumOptimizerWithGRFPenalizedSmootherNativeInput();

      outputRho = new DenseMatrix64F(rhoSize, 1);
      rhoPrevious = new DenseMatrix64F(rhoSize, 1);
      
      rhoPreviousYoMatrix = new YoMatrix("rhoPreviousYoMatrix", rhoSize, 1, registry);
   }

   public void setRateOfChangeOfGroundReactionForceRegularization(DenseMatrix64F wRhoSmoother)
   {
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setRateOfChangeOfGroundReactionForceRegularization(wRhoSmoother);
   }

   public void reset()
   {
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.reset();
   }

   public int getRhoSize()
   {
      return rhoSize;
   }

   public int getNSupportVectors()
   {
      return nSupportVectors;
   }

   public int getNPointsPerPlane()
   {
      return nPointsPerPlane;
   }

   public int getNPlanes()
   {
      return nPlanes;
   }

   public void setInputs(
         DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F momentumDotWeight, 
                         DenseMatrix64F jSecondary, DenseMatrix64F pSecondary, DenseMatrix64F weightMatrixSecondary,
                         DenseMatrix64F WRho, DenseMatrix64F Lambda,
                         DenseMatrix64F WRhoSmoother, DenseMatrix64F rhoPrevAvg, DenseMatrix64F WRhoCop,
                         DenseMatrix64F QRho, DenseMatrix64F c,
                         DenseMatrix64F rhoMin, DenseMatrix64F QfeetCoP
                         )
   {
      //A,b,c
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setCentroidalMomentumMatrix(a);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setMomentumDotEquationRightHandSide(b);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setMomentumDotWeight(momentumDotWeight);

      //Js ps Ws
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setSecondaryConstraintJacobian(jSecondary);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setSecondaryConstraintRightHandSide(pSecondary);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setSecondaryConstraintWeight(weightMatrixSecondary);

      //Wrho, Lambda
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setGroundReactionForceRegularization(WRho);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setJointAccelerationRegularization(Lambda);

      //WRho, Rho_pavg , WRhoCop
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setRateOfChangeOfGroundReactionForceRegularization(WRhoSmoother);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setRhoPreviousAverage(rhoPrevAvg);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setCenterOfPressurePenalizedRegularization(WRhoCop);

      //QRho,c 
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setContactPointWrenchMatrix(QRho);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setWrenchEquationRightHandSide(c);
      
      //Rho_min
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setRhoMin(rhoMin);
      
      //Wrho
      rhoPreviousYoMatrix.get(rhoPrevious);
      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setRhoPrevious(rhoPrevious);
      
      //QfeetCoP
//      momentumOptimizerWithGRFPenalizedSmootherNativeInput.setQfeetCoP(QfeetCoP);
   }

   
   public int solve() throws NoConvergenceException
   {
      CVXMomentumOptimizerWithGRFPenalizedSmootherNativeOutput momentumOptimizerWithGRFPenalizedSmootherNativeOutput;
      int ret=-999;

      try
      {
         ret=momentumOptimizerWithGRFPenalizedSmootherNative.solve(momentumOptimizerWithGRFPenalizedSmootherNativeInput);
      }
      finally
      {
         momentumOptimizerWithGRFPenalizedSmootherNativeOutput=momentumOptimizerWithGRFPenalizedSmootherNative.getOutput();
         outputRho = momentumOptimizerWithGRFPenalizedSmootherNativeOutput.getRho();
         outputJointAccelerations = momentumOptimizerWithGRFPenalizedSmootherNativeOutput.getJointAccelerations();
         outputOptVal = momentumOptimizerWithGRFPenalizedSmootherNativeOutput.getOptVal();
         
         rhoPreviousYoMatrix.set(outputRho);
      }
      
      if (ret<0)
         throw new NoConvergenceException();
      
      return ret;
   }
   

   public DenseMatrix64F getOutputRho()
   {
      return outputRho;
   }

   public DenseMatrix64F getOutputJointAccelerations()
   {
      return outputJointAccelerations;
   }

   public double getOutputOptVal()
   {
      return outputOptVal;
   }

   @Override
   public void setInputs(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F momentumDotWeight, DenseMatrix64F jPrimary, DenseMatrix64F pPrimary,
         DenseMatrix64F jSecondary, DenseMatrix64F pSecondary, DenseMatrix64F weightMatrixSecondary, DenseMatrix64F WRho, DenseMatrix64F Lambda,
         DenseMatrix64F RhoSmoother, DenseMatrix64F rhoPrevAvg, DenseMatrix64F WRhoCop, DenseMatrix64F QRho, DenseMatrix64F c, DenseMatrix64F rhoMin, DenseMatrix64F QfeetCoP)
   {
      throw new RuntimeException("Not implemented, current CVXGEN version do not take primary constraints");
      
   }
}
