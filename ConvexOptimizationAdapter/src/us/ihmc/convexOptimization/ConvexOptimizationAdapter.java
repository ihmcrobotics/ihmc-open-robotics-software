package us.ihmc.convexOptimization;

public interface ConvexOptimizationAdapter
{

   public abstract void setLinearCostFunctionVector(double[] linearCostFunctionVector);

   public abstract void setQuadraticCostFunction(double[][] quadraticCostFunctionPMatrix, double[] quadraticCostFunctionQVector, double quadraticCostFunctionR);

   public abstract void setLinearEqualityConstraintsAMatrix(double[][] linearEqualityConstraintsAMatrix);
   
   public abstract void setLinearEqualityConstraintsBVector(double[] linearEqualityConstraintsBVector);
   
   public abstract void setLinearInequalityConstraints(double[][] linearInequalityConstraintCVectors, double[] linearInequalityConstraintBs);
   
   public abstract double[] solve();
   
   public abstract void dispose();
}

