package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.wrenchDistribution.PlaneContactWrenchMatrixCalculator;
import us.ihmc.utilities.exeptions.NoConvergenceException;

public interface MomentumOptimizerInterface
{

   public abstract void setRateOfChangeOfGroundReactionForceRegularization(DenseMatrix64F wRhoSmoother);

   public abstract void reset();

   public abstract int getRhoSize();

   public abstract int getNSupportVectors();

   public abstract int getNPointsPerPlane();

   public abstract int getNPlanes();

   public abstract void setInputs(DenseMatrix64F a, DenseMatrix64F b, PlaneContactWrenchMatrixCalculator wrenchMatrixCalculator,
         DenseMatrix64F wrenchEquationRightHandSide, DenseMatrix64F momentumDotWeight, DenseMatrix64F dampedLeastSquaresFactorMatrix,
         DenseMatrix64F jSecondary, DenseMatrix64F pSecondary, DenseMatrix64F weightMatrixSecondary);

   public abstract int solve() throws NoConvergenceException;

   public abstract DenseMatrix64F getOutputRho();

   public abstract DenseMatrix64F getOutputJointAccelerations();

   public abstract double getOutputOptVal();

}