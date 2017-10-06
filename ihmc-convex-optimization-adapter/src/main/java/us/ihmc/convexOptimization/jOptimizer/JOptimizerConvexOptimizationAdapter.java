package us.ihmc.convexOptimization.jOptimizer;


import java.util.ArrayList;
import java.util.List;

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

import com.joptimizer.functions.ConvexMultivariateRealFunction;
import com.joptimizer.functions.LinearMultivariateRealFunction;
import com.joptimizer.functions.SOCPLogarithmicBarrier;
import com.joptimizer.functions.SOCPLogarithmicBarrier.SOCPConstraintParameters;
import com.joptimizer.optimizers.BarrierMethod;
import com.joptimizer.optimizers.JOptimizer;
import com.joptimizer.optimizers.OptimizationRequest;
import com.joptimizer.optimizers.OptimizationResponse;

import us.ihmc.convexOptimization.ConvexOptimizationAdapter;

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
   
   // Second order cone constraints
   private List<SOCPConstraintParameters> socpConstraintParameterList = new ArrayList<SOCPLogarithmicBarrier.SOCPConstraintParameters>();
   
   private int variableDimension;
   
   
   public JOptimizerConvexOptimizationAdapter() 
   {
      this.variableDimension=-1;
   }
   
   public JOptimizerConvexOptimizationAdapter(int variableDimension) 
   {
      assert(variableDimension>0);
      this.variableDimension = variableDimension;
   }
   
   private void checkDimension(int variableDimension)
   {
      if(this.variableDimension<0)
      {
         this.variableDimension = variableDimension;
      }
      else if(this.variableDimension!=variableDimension)
      {
         new Exception("Incorrect constraint dimension, problem variableDimension was previosly inferred as"+ this.variableDimension);
      }
   }
   
   public void setLinearCostFunctionVector(double[] linearCostFunctionFVector)
   {
      checkDimension(linearCostFunctionFVector.length);
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
      if(linearEqualityConstraintsAMatrix.length==0)
      {
         System.err.println("Warning: adding linearEqualityConstraintsMatrix with zero row");
         return ;
      }
      checkDimension(linearEqualityConstraintsAMatrix[0].length);
      this.linearEqualityConstraintsAMatrix = linearEqualityConstraintsAMatrix;
   }
   

   public void setLinearEqualityConstraintsBVector(double[] linearEqualityConstraintsBVector)
   {
      checkDimension(linearEqualityConstraintsBVector.length);
      this.linearEqualityConstraintsBVector = linearEqualityConstraintsBVector;
   }

   public void setLinearInequalityConstraints(double[][] linearInequalityConstraintCVectors, double[] linearInequalityConstraintBs)
   {
      for(int i=0;i<linearInequalityConstraintCVectors.length;i++)
        checkDimension(linearInequalityConstraintCVectors[i].length);

      this.linearInequalityConstraintCVectors = linearInequalityConstraintCVectors;
      this.linearInequalityConstraintBs = linearInequalityConstraintBs;
   }
   
   private double[] findFeasiblePointUsingApacheSimplexSolver()
   {
      //dummy cost function
      LinearObjectiveFunction f = new LinearObjectiveFunction(new double[linearCostFunctionFVector.length], 0.);
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

   public void removeRedundantLinearEqualityConstraint() 
   {
      if(linearEqualityConstraintsAMatrix==null)
         return;
      RealMatrix equalityConstraintMatrix = new Array2DRowRealMatrix(linearEqualityConstraintsAMatrix);
      RRQRDecomposition equalityConstraintQRDecomposition= new RRQRDecomposition(equalityConstraintMatrix);
      RRQRDecomposition equalityConstraintTransposeQRDecomposition= new RRQRDecomposition(equalityConstraintMatrix.transpose());

      int[] nonZeroIndexPerRow = firstNonZeroValueEachRow(equalityConstraintTransposeQRDecomposition.getP());
//      System.out.println("Pindex:="+Arrays.toString(nonZeroIndexPerRow));
      
      //check consistency of dependent constraints
      final int rank=equalityConstraintTransposeQRDecomposition.getRank(1e-10);
      for(int i=rank; i<linearEqualityConstraintsAMatrix.length;i++)
      {

         double residual=equalityConstraintQRDecomposition.getQ().getColumnVector(i).dotProduct(new ArrayRealVector(linearEqualityConstraintsBVector));
         if(Math.abs(residual)<1e-10)
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
      
      removeRedundantLinearEqualityConstraint();
      // SetUp JOptimizer
      OptimizationRequest optimizationRequest = new OptimizationRequest();
      
      double objectiveFunctionRScalar = 0.0;
      
      //linear only
      LinearMultivariateRealFunction objectiveFunction = new LinearMultivariateRealFunction(this.linearCostFunctionFVector, objectiveFunctionRScalar);
      optimizationRequest.setF0(objectiveFunction);

      
      if (this.linearEqualityConstraintsAMatrix != null)
      {
        optimizationRequest.setA(this.linearEqualityConstraintsAMatrix);
        optimizationRequest.setB(this.linearEqualityConstraintsBVector);
      }

      OptimizationResponse optimizationResponse;
      try{
        if(socpConstraintParameterList.size()>0)
        {
          optimizationResponse=solveSOCP(optimizationRequest);
        }
        else
        {
          optimizationResponse=solveQP(optimizationRequest);
        }
      }
      catch (Exception e)
      {
         System.out.println("unable to solve problem");
         return null;
      }

      return optimizationResponse.getSolution();    
   }
   
   private void convertLinearInequalityConstraintsToSecondOrderConeConstraints()
   {
      double[][] negLinearInequalityConstraintCVectors = new double[linearInequalityConstraintCVectors.length][];
      for(int i=0;i<linearInequalityConstraintCVectors.length;i++)
      {
          negLinearInequalityConstraintCVectors[i] = new double[linearInequalityConstraintCVectors[i].length];
          for(int j=0;j<linearInequalityConstraintCVectors[i].length;j++)
              negLinearInequalityConstraintCVectors[i][j]=-linearInequalityConstraintCVectors[i][j];
          addSecondOrderConeConstraints(new double[1][variableDimension], new double[1], negLinearInequalityConstraintCVectors[i],linearInequalityConstraintBs[i]);
      }
   }
   

   private OptimizationResponse solveSOCP(OptimizationRequest optimizationRequest) throws Exception
   {
      
      List<SOCPConstraintParameters> socpConstraintParametersList=this.socpConstraintParameterList;
      SOCPLogarithmicBarrier barrierFunction = new SOCPLogarithmicBarrier(socpConstraintParametersList, 3);
      convertLinearInequalityConstraintsToSecondOrderConeConstraints();
      BarrierMethod jOptimizer = new BarrierMethod(barrierFunction);
      jOptimizer.setOptimizationRequest(optimizationRequest);
      int returnCode = jOptimizer.optimize();
      return jOptimizer.getOptimizationResponse();
      
   }
   
   
   private OptimizationResponse solveQP(OptimizationRequest optimizationRequest) throws Exception
   {
        ConvexMultivariateRealFunction[] inequalities = convertAllInequalityVectorsToJOptimizerConstraints(this.linearInequalityConstraintCVectors, this.linearInequalityConstraintBs);
        if ((inequalities != null) && (inequalities.length != 0))
        {
          optimizationRequest.setFi(inequalities);
        }
      optimizationRequest.setToleranceFeas(1.0e-8);
      optimizationRequest.setTolerance(2.0e-8);
      optimizationRequest.setMaxIteration(500);

      optimizationRequest.setInitialPoint(findFeasiblePointUsingApacheSimplexSolver());
//      optimizationRequest.setNotFeasibleInitialPoint(new double[linearCostFunctionFVector.length]);
      
      // optimization

      JOptimizer jOptimizer = new JOptimizer();
      jOptimizer.setOptimizationRequest(optimizationRequest);

      int returnCode;
      returnCode = jOptimizer.optimize();

      return jOptimizer.getOptimizationResponse();
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

   public void addSecondOrderConeConstraints(double[][] secondOrderConeAMatrix, double[] secondOrderConeBVector, double[] secondOrderConeCVector,
         double secondOrderConeDScalar)
   {
      for(int i=0;i<secondOrderConeAMatrix.length;i++)
         checkDimension(secondOrderConeAMatrix[i].length);
      checkDimension(secondOrderConeCVector.length);

      List<SOCPConstraintParameters>socpConstraintParametersList=this.socpConstraintParameterList;
      SOCPLogarithmicBarrier barrierFunction = new SOCPLogarithmicBarrier(socpConstraintParametersList, this.variableDimension);
      
      SOCPConstraintParameters constraintParams = barrierFunction.new SOCPConstraintParameters(secondOrderConeAMatrix, secondOrderConeBVector, secondOrderConeCVector, secondOrderConeDScalar);
      this.socpConstraintParameterList.add(socpConstraintParametersList.size(), constraintParams);
      
   }
   
}

