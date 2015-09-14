package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.tools.exceptions.NoConvergenceException;

public interface MomentumOptimizerInterface 
{

   public abstract void setRateOfChangeOfGroundReactionForceRegularization(DenseMatrix64F wRhoSmoother);

   public abstract void reset();

   public abstract int getRhoSize();

   public abstract int getNSupportVectors();

   public abstract int getNPointsPerPlane();

   public abstract int getNPlanes();

//   public abstract void setInputs(DenseMatrix64F a, DenseMatrix64F b, PlaneContactWrenchMatrixCalculator wrenchMatrixCalculator,
//         DenseMatrix64F wrenchEquationRightHandSide, DenseMatrix64F momentumDotWeight, DenseMatrix64F dampedLeastSquaresFactorMatrix,
//         DenseMatrix64F jSecondary, DenseMatrix64F pSecondary, DenseMatrix64F weightMatrixSecondary);
   
   public abstract void setInputs(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F momentumDotWeight, 
                         DenseMatrix64F jSecondary, DenseMatrix64F pSecondary, DenseMatrix64F weightMatrixSecondary,
                         DenseMatrix64F WRho, DenseMatrix64F Lambda,
                         DenseMatrix64F RhoSmoother, DenseMatrix64F rhoPrevAvg, DenseMatrix64F WRhoCop,
                         DenseMatrix64F QRho, DenseMatrix64F c,
                         DenseMatrix64F rhoMin, DenseMatrix64F QfeetCoP
                         );

   public abstract int solve() throws NoConvergenceException;

   public abstract DenseMatrix64F getOutputRho();

   public abstract DenseMatrix64F getOutputJointAccelerations();

   public abstract double getOutputOptVal();

//   public abstract void setInputs(DenseMatrix64F a, DenseMatrix64F b, PlaneContactWrenchMatrixCalculator wrenchMatrixCalculator,
//         DenseMatrix64F wrenchEquationRightHandSide, DenseMatrix64F momentumDotWeight, DenseMatrix64F dampedLeastSquaresFactorMatrix, DenseMatrix64F jPrimary,
//         DenseMatrix64F pPrimary, DenseMatrix64F jSecondary, DenseMatrix64F pSecondary, DenseMatrix64F weightMatrixSecondary);
   public abstract void setInputs(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F momentumDotWeight, 
                         DenseMatrix64F jPrimary, DenseMatrix64F pPrimary, 
                         DenseMatrix64F jSecondary, DenseMatrix64F pSecondary, DenseMatrix64F weightMatrixSecondary,
                         DenseMatrix64F WRho, DenseMatrix64F Lambda,
                         DenseMatrix64F RhoSmoother, DenseMatrix64F rhoPrevAvg, DenseMatrix64F WRhoCop,
                         DenseMatrix64F QRho, DenseMatrix64F c,
                         DenseMatrix64F rhoMin, DenseMatrix64F QfeetCoP
                         );


}