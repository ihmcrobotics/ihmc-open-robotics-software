package us.ihmc.convexOptimization.jOptimizer;


import java.util.ArrayList;
import java.util.Arrays;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RRQRDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.optim.MaxIter;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.linear.LinearConstraint;
import org.apache.commons.math3.optim.linear.LinearConstraintSet;
import org.apache.commons.math3.optim.linear.LinearObjectiveFunction;
import org.apache.commons.math3.optim.linear.Relationship;
import org.apache.commons.math3.optim.linear.SimplexSolver;

import us.ihmc.convexOptimization.ConvexOptimizationAdapter;

import com.joptimizer.functions.ConvexMultivariateRealFunction;
import com.joptimizer.functions.LinearMultivariateRealFunction;
import com.joptimizer.optimizers.JOptimizer;
import com.joptimizer.optimizers.OptimizationRequest;
import com.joptimizer.optimizers.OptimizationResponse;

public class JOptimizerConvexOptimizationAdapter implements ConvexOptimizationAdapter
{
   /**
    *  f(x) = x'Px + q'x  + r
    *  st  Ax =b
    *      Cx <= bs.
    *      
    */
   
   // Cost function to be minimized.
   private double[] linearCostFunctionFVector;
   
   private double[][] quadraticCostFunctionPMatrix;
   private double[] quadraticCostFunctionQVector;
   private double quadraticCostFunctionR;
   
   // Linear Equality Constraints.
   private double[][] linearEqualityConstraintsAMatrix;
   private double[] linearEqualityConstraintsBVector;
   
   // Linear Inequality Constraints.
   private double[][] linearInequalityConstraintCVectors;
   private double[] linearInequalityConstraintBs;
   
   
   
   public JOptimizerConvexOptimizationAdapter() 
   {
   }
   
   
   public void setLinearCostFunctionVector(double[] linearCostFunctionFVector)
   {
      this.linearCostFunctionFVector = linearCostFunctionFVector;      
   }

   public void setQuadraticCostFunction(double[][] quadraticCostFunctionPMatrix, double[] quadraticCostFunctionQVector, double quadraticCostFunctionR)
   {
      this.quadraticCostFunctionPMatrix = quadraticCostFunctionPMatrix;
      this.quadraticCostFunctionQVector = quadraticCostFunctionQVector;
      this.quadraticCostFunctionR = quadraticCostFunctionR;  
      
      throw new RuntimeException("Not implemented yet!");
   }

   public void setLinearEqualityConstraintsAMatrix(double[][] linearEqualityConstraintsAMatrix)
   {
      this.linearEqualityConstraintsAMatrix = linearEqualityConstraintsAMatrix;
   }
   

   public void setLinearEqualityConstraintsBVector(double[] linearEqualityConstraintsBVector)
   {
      this.linearEqualityConstraintsBVector = linearEqualityConstraintsBVector;
   }

   public void setLinearInequalityConstraints(double[][] linearInequalityConstraintCVectors, double[] linearInequalityConstraintBs)
   {
      this.linearInequalityConstraintCVectors = linearInequalityConstraintCVectors;
      this.linearInequalityConstraintBs = linearInequalityConstraintBs;
   }
   
   private double[] findFeasiblePointUsingApacheSimplexSolver()
   {
      LinearObjectiveFunction f = new LinearObjectiveFunction(linearCostFunctionFVector, 0.);
      ArrayList<LinearConstraint> constraints = new ArrayList<LinearConstraint>();

      if(linearEqualityConstraintsAMatrix!=null)
      {
        for(int i=0;i<linearEqualityConstraintsBVector.length;i++)
        {
          constraints.add(new LinearConstraint(linearEqualityConstraintsAMatrix[i], Relationship.EQ, linearEqualityConstraintsBVector[i]));
        }
      }

      final double eps=1e-10;
      if(linearInequalityConstraintCVectors!=null)
      {
        for(int i=0;i<linearInequalityConstraintBs.length;i++)
        {
          constraints.add(new LinearConstraint(linearInequalityConstraintCVectors[i], Relationship.LEQ, linearInequalityConstraintBs[i]-eps));
        }
      }
      
      PointValuePair solution = new SimplexSolver().optimize(f, new LinearConstraintSet(constraints), new MaxIter(Integer.MAX_VALUE));
      return solution.getPoint();
   }
   
   private static int[] firstNonZeroValueEachRow(RealMatrix m)
   {
      int[] nonZeroIndexes =new int[m.getRowDimension()];

      for(int rowIndex=0;rowIndex<m.getRowDimension();rowIndex++)
      {
         nonZeroIndexes[rowIndex]=-1;
         for(int colIndex=0;colIndex<m.getColumnDimension();colIndex++)
            if(m.getEntry(rowIndex,colIndex)>0)
            {
               nonZeroIndexes[rowIndex]=colIndex;
               break;
            }
      }
      return nonZeroIndexes;
   }

   public void removeRedundantEqualityConstraint() 
   {
      if(linearEqualityConstraintsAMatrix==null)
         return;
      RealMatrix equalityConstraintMatrix = new Array2DRowRealMatrix(linearEqualityConstraintsAMatrix);
      RRQRDecomposition equalityConstraintQRDecomposition= new RRQRDecomposition(equalityConstraintMatrix);
      RRQRDecomposition equalityConstraintTransposeQRDecomposition= new RRQRDecomposition(equalityConstraintMatrix.transpose());

      int[] nonZeroIndexPerRow = firstNonZeroValueEachRow(equalityConstraintTransposeQRDecomposition.getP());
      System.out.println("Pindex:="+Arrays.toString(nonZeroIndexPerRow));
      
      //check consistency of dependent constraints
      final int rank=equalityConstraintTransposeQRDecomposition.getRank(1e-10);
      for(int i=rank; i<linearEqualityConstraintsAMatrix.length;i++)
      {

         double residual=equalityConstraintQRDecomposition.getQ().getColumnVector(i).dotProduct(new ArrayRealVector(linearEqualityConstraintsBVector));
         if(residual<1e-10)
         {
            System.out.println("Redundant Constraints "+nonZeroIndexPerRow[i]+" residual="+residual);
         }
         else
         {
            System.err.println("Inconsistent Constraints "+nonZeroIndexPerRow[i]+" residual="+residual + "proceed without them");
         }
      }
      
      //keep independent ones
      double[][] newLinearEqualityConstraintsAMatrix =new double[rank][];
      double[] newLinearEqualityConstraintsBVector = new double[rank];
      for(int i=0;i<rank;i++)
      {
         newLinearEqualityConstraintsAMatrix[i]=linearEqualityConstraintsAMatrix[nonZeroIndexPerRow[i]];
         newLinearEqualityConstraintsBVector[i]=linearEqualityConstraintsBVector[nonZeroIndexPerRow[i]];
      }
      
      setLinearEqualityConstraintsAMatrix(newLinearEqualityConstraintsAMatrix);
      setLinearEqualityConstraintsBVector(newLinearEqualityConstraintsBVector);

   }
   

   public double[] solve()
   {
      removeRedundantEqualityConstraint();
      // SetUp JOptimizer
      OptimizationRequest optimizationRequest = new OptimizationRequest();
      
      double objectiveFunctionRScalar = 0.0;
      LinearMultivariateRealFunction objectiveFunction = new LinearMultivariateRealFunction(this.linearCostFunctionFVector, objectiveFunctionRScalar);
      optimizationRequest.setF0(objectiveFunction);

      
      if (this.linearEqualityConstraintsAMatrix != null)
      {
         optimizationRequest.setA(this.linearEqualityConstraintsAMatrix);
         optimizationRequest.setB(this.linearEqualityConstraintsBVector);
      }
      
      ConvexMultivariateRealFunction[] inequalities = convertAllInequalityVectorsToJOptimizerConstraints(this.linearInequalityConstraintCVectors, this.linearInequalityConstraintBs);
      if ((inequalities != null) && (inequalities.length != 0))
      {
         optimizationRequest.setFi(inequalities);
      }
      optimizationRequest.setInitialPoint(findFeasiblePointUsingApacheSimplexSolver());
//      optimizationRequest.setNotFeasibleInitialPoint(new double[linearCostFunctionFVector.length]);
      
      optimizationRequest.setToleranceFeas(1.0e-8);
      optimizationRequest.setTolerance(2.0e-8);
      optimizationRequest.setMaxIteration(500);

      // optimization
      JOptimizer jOptimizer = new JOptimizer();
      jOptimizer.setOptimizationRequest(optimizationRequest);

      int returnCode;
      try
      {
         returnCode = jOptimizer.optimize();
      }
      catch (Exception e)
      {
         System.err.println("***"+getClass().getSimpleName()+":"+e.getMessage());
         return null;
      }

      if (returnCode == OptimizationResponse.FAILED)
      {
         return null;
      }

      OptimizationResponse optimizationResponse = jOptimizer.getOptimizationResponse();
      return optimizationResponse.getSolution();    
   }
   
   private static ConvexMultivariateRealFunction[] convertAllInequalityVectorsToJOptimizerConstraints(double[][] linearInequalityConstraintCVectors, double[] linearInequalityConstraintBs)
   {
      if (linearInequalityConstraintCVectors == null) return new ConvexMultivariateRealFunction[0];
      
      ConvexMultivariateRealFunction[] ret = new ConvexMultivariateRealFunction[linearInequalityConstraintCVectors.length];

      for (int i=0; i<linearInequalityConstraintCVectors.length; i++)
      {
         ret[i] = new LinearMultivariateRealFunction(linearInequalityConstraintCVectors[i], -linearInequalityConstraintBs[i]);
      }
      return ret;
   }

   public void dispose()
   {
   }

   public void addQuadraticInequalities(double[][] pMatrix, double[] qVector, double r)
   {
      throw new RuntimeException("Not yet implemented");
   }

   public void addSecondOrderConeConstraints(double[][] secondOrderConeAMatrix, double secondOrderConeBScalar, double[] secondOrderConeCVector,
         double secondOrderConeDScalar)
   {
      throw new RuntimeException("Not yet implemented");      
   }
   
}

