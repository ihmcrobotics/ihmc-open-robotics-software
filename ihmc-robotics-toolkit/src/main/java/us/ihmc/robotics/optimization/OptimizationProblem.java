package us.ihmc.robotics.optimization;

import us.ihmc.robotics.optimization.constrainedOptimization.ConstraintFunction;

import java.util.List;

public interface OptimizationProblem
{
   /**
    *
    * @return list of bounds, or null if not bounded
    */
   public RealDomainBounds[] getDomain();

   public CostFunction getCostFunction();

   public List<ConstraintFunction> getEqualityConstraints();
   public List<ConstraintFunction> getInequalityConstraints();
}
