package us.ihmc.convexOptimization;

public interface ConvexOptimizationAdapter
{
   public abstract void setLinearCostFunctionVector(double[] linearCostFunctionVector);

   public abstract void setQuadraticCostFunction(double[][] quadraticCostFunctionPMatrix, double[] quadraticCostFunctionQVector, double quadraticCostFunctionR);

   public abstract void setLinearEqualityConstraintsAMatrix(double[][] linearEqualityConstraintsAMatrix);
   
   public abstract void setLinearEqualityConstraintsBVector(double[] linearEqualityConstraintsBVector);
   
   public abstract void setLinearInequalityConstraints(double[][] linearInequalityConstraintCVectors, double[] linearInequalityConstraintBs);
   
   public abstract void addQuadraticInequalities(double[][] quadraticPMatrix, double[] quadraticQVector, double quadraticRScalar);

   public abstract void addSecondOrderConeConstraints(double[][] secondOrderConeAMatrix, double[] secondOrderConeBVector, double[] secondOrderConeCVector, double secondOrderConeDScalar);
   
   public abstract double[] solve();
   
   public abstract void dispose();
}

