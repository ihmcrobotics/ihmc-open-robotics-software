package us.ihmc.convexOptimization.jOptimizer;

import us.ihmc.convexOptimization.ConvexOptimizationAdapter;

import com.joptimizer.functions.ConvexMultivariateRealFunction;
import com.joptimizer.functions.LinearMultivariateRealFunction;
import com.joptimizer.optimizers.JOptimizer;
import com.joptimizer.optimizers.OptimizationRequest;
import com.joptimizer.optimizers.OptimizationResponse;

public class JOptimizerConvexOptimizationAdapter implements ConvexOptimizationAdapter
{
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
   

   public double[] solve() 
   {
      // SetUp JOptimizer
      OptimizationRequest optimizationRequest = new OptimizationRequest();
      
      double objectiveFunctionRScalar = 0.0;
      LinearMultivariateRealFunction objectiveFunction = new LinearMultivariateRealFunction(this.linearCostFunctionFVector, objectiveFunctionRScalar);
      optimizationRequest.setF0(objectiveFunction);

      // or.setInitialPoint(new double[] { 0., 0.});
      
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
      
      optimizationRequest.setToleranceFeas(1.0e-6);
      optimizationRequest.setTolerance(2.0e-6);
      optimizationRequest.setMaxIteration(50); //500);

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

   public void addSecondOrderConeConstraints(double[][] secondOrderConeAMatrix, double[] secondOrderConeBVector, double[] secondOrderConeCVector,
         double secondOrderConeDScalar)
   {
      throw new RuntimeException("Not yet implemented");      
   }
   
}

