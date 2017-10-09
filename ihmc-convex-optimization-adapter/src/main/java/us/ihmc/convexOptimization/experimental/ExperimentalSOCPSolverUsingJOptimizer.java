package us.ihmc.convexOptimization.experimental;


import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import com.joptimizer.functions.ConvexMultivariateRealFunction;
import com.joptimizer.functions.LinearMultivariateRealFunction;
import com.joptimizer.functions.PSDQuadraticMultivariateRealFunction;
import com.joptimizer.optimizers.JOptimizer;
import com.joptimizer.optimizers.OptimizationRequest;
import com.joptimizer.optimizers.OptimizationResponse;

public class ExperimentalSOCPSolverUsingJOptimizer implements ExperimentalSOCPSolver
{
   JOptimizer jOptimizer = new JOptimizer();
   OptimizationRequest optimizationRequest = new OptimizationRequest();
   
   public void setOptimizationFunctionVectorF(double[] optimizationFunctionVectorF)
   {
      LinearMultivariateRealFunction linearFunctionToOptimize = new LinearMultivariateRealFunction(optimizationFunctionVectorF, 0.0);
      optimizationRequest.setF0(linearFunctionToOptimize);
   }

   public void setOptimizationFunctionVectorF(DenseMatrix64F optimizationFunctionVectorF)
   {
      setOptimizationFunctionVectorF(optimizationFunctionVectorF.getData());      
   }

   public void setLinearEqualityConstraints(double[][] linearEqualityAMatrix, double[] linearEqualityBVector)
   {
      optimizationRequest.setA(linearEqualityAMatrix);
      optimizationRequest.setB(linearEqualityBVector);
   }

   public void setLinearEqualityConstraints(DenseMatrix64F linearEqualityAMatrix, DenseMatrix64F linearEqualityBVector)
   {
      setLinearEqualityConstraints(convertMatrixToTwoDimensionalDoubleArray(linearEqualityAMatrix), linearEqualityBVector.getData());
   }
   
   public void setSpecialSecondOrderConeInequality(double[][] coneInequalityMatrixB, double[] coneInequalityVectorU, ArrayList<ConvexMultivariateRealFunction> otherInequalities)
   {
      int numberOfRows = coneInequalityMatrixB.length;
      int numberOfColumns = coneInequalityMatrixB[0].length;
      
      if (numberOfRows != numberOfColumns)
      {
         throw new RuntimeException("coneInequalityMatrixB must be square!");
      }
      
      if (coneInequalityVectorU.length != numberOfRows)
      {
         throw new RuntimeException("coneInequalityVectorU must be have correct length!");
      }
      
      DenseMatrix64F coneInequalityDenseMatrixB = new DenseMatrix64F(coneInequalityMatrixB);
      DenseMatrix64F coneInequalityVectorUAsDenseMatrix = new DenseMatrix64F(numberOfRows, 1);
      
      coneInequalityVectorUAsDenseMatrix.setData(coneInequalityVectorU);
      
      setSpecialSecondOrderConeInequality(coneInequalityDenseMatrixB, coneInequalityVectorUAsDenseMatrix, otherInequalities);
   }

   public void setSpecialSecondOrderConeInequality(DenseMatrix64F coneInequalityMatrixB, DenseMatrix64F coneInequalityVectorU, ArrayList<ConvexMultivariateRealFunction> otherInequalities)
   {
      int numberOfRows = coneInequalityMatrixB.getNumRows();
      int numberOfColumns = coneInequalityMatrixB.getNumCols();
      
      if (numberOfRows != numberOfColumns)
      {
         throw new RuntimeException("coneInequalityMatrixB must be square!");
      }

      if (coneInequalityVectorU.getNumRows() != numberOfRows)
      {
         throw new RuntimeException("coneInequalityVectorU must be the correct size!");
      }
      
      if (coneInequalityVectorU.getNumCols() != 1)
      {
         throw new RuntimeException("coneInequalityVectorU must have one column!");
      }
      
      ConvexMultivariateRealFunction[] inequalities = new ConvexMultivariateRealFunction[otherInequalities.size() + 2];

      // Add the quadratic inequality x^T ( B^TB - uu^T) x <= 0

      DenseMatrix64F coneInequalityMatrixBTransposeB = new DenseMatrix64F(numberOfRows, numberOfColumns);
      CommonOps.multTransA(coneInequalityMatrixB, coneInequalityMatrixB, coneInequalityMatrixBTransposeB);
      
      DenseMatrix64F coneInequalityMatrixUUTranspose = new DenseMatrix64F(numberOfRows, numberOfColumns);
      CommonOps.multTransB(coneInequalityVectorU, coneInequalityVectorU, coneInequalityMatrixUUTranspose);

      DenseMatrix64F quadraticInequalityMatrixP = new DenseMatrix64F(numberOfRows, numberOfColumns);      
      CommonOps.subtract(coneInequalityMatrixBTransposeB, coneInequalityMatrixUUTranspose, quadraticInequalityMatrixP);
      CommonOps.scale(2.0, quadraticInequalityMatrixP);

      double[][] quadraticInequalityMatrixPAsDoubleArray = convertMatrixToTwoDimensionalDoubleArray(quadraticInequalityMatrixP);
  
      double[] qVector = new double[numberOfRows];
      double r = 0.0;
      ConvexMultivariateRealFunction  quadraticInequalityFunction = new PSDQuadraticMultivariateRealFunction(quadraticInequalityMatrixPAsDoubleArray, qVector, r);    
      inequalities[0] = quadraticInequalityFunction;
      
      // Add the linear inequality -u^T x <= 0
      qVector = new double[numberOfRows];
      for (int row=0; row<numberOfRows; row++)
      {
         qVector[row] = -coneInequalityVectorU.get(row, 0);
      }
      r = 0.0;
      LinearMultivariateRealFunction linearInequalityFunction = new LinearMultivariateRealFunction(qVector, r);
      inequalities[1] = linearInequalityFunction;

      for (int i=0; i<otherInequalities.size(); i++)
      {
         inequalities[i+2] = otherInequalities.get(i);
      }
      
      optimizationRequest.setFi(inequalities);
   }

   public double[] solveAndReturnOptimalVector()
   {
      optimizationRequest.setToleranceFeas(1.E-6);
      optimizationRequest.setTolerance(2.E-6);
      optimizationRequest.setMaxIteration(500);
      //optimizationRequest.setInitialPoint(new double[]{-0.5,4.5});
      //optimizationRequest.setNotFeasibleInitialPoint(new double[]{4,0});
      //optimizationRequest.setInteriorPointMethod(JOptimizer.BARRIER_METHOD);
      //optimizationRequest.setCheckKKTSolutionAccuracy(true);

      // optimization
      jOptimizer.setOptimizationRequest(optimizationRequest);
      try
      {
         jOptimizer.optimize();
      } 
      catch (Exception e)
      {
         return null;
      }

      OptimizationResponse response = jOptimizer.getOptimizationResponse();

      if (response.getReturnCode() == OptimizationResponse.FAILED)
      {
         return null;
      }

      return response.getSolution();
   }
   
   
   private double[][] convertMatrixToTwoDimensionalDoubleArray(DenseMatrix64F matrix)
   {
      int numberOfRows = matrix.getNumRows();
      int numberOfColumns = matrix.getNumCols();
      
      double[][] returnDoubleArray = new double[numberOfRows][numberOfColumns];
      
      for (int row = 0; row < numberOfRows; row++)
      {
         for (int column = 0; column < numberOfColumns; column++)
         {
            returnDoubleArray[row][column] = matrix.get(row, column);
         }
      }
      
      return returnDoubleArray;
   }

}
