package us.ihmc.convexOptimization.experimental;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;

import com.joptimizer.functions.ConvexMultivariateRealFunction;

public interface ExperimentalSOCPSolver
{
   // Minimize f^T x
   public void setOptimizationFunctionVectorF(double[] optimizationFunctionVectorF);
   public void setOptimizationFunctionVectorF(DenseMatrix64F optimizationFunctionVectorF);
   
   /*
    * A x = b
    */
   public void setLinearEqualityConstraints(double[][] linearEqualityAMatrix, double[] linearEqualityBVector);
   public void setLinearEqualityConstraints(DenseMatrix64F linearEqualityAMatrix, DenseMatrix64F linearEqualityBVector);
   
   
   /*
    * || B x || <= u^T x
    */
   public void setSpecialSecondOrderConeInequality(double[][] coneInequalityMatrixB, double[] coneInequalityVectorU, ArrayList<ConvexMultivariateRealFunction> otherInequalities);
   public void setSpecialSecondOrderConeInequality(DenseMatrix64F coneInequalityMatrixB, DenseMatrix64F coneInequalityVectorU, ArrayList<ConvexMultivariateRealFunction> otherInequalities);
   
   public double[] solveAndReturnOptimalVector();
}
