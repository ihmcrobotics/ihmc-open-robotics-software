package us.ihmc.robotics.optimization.EvaluationProblems;

import org.ejml.data.DMatrixD1;
import us.ihmc.robotics.optimization.CostFunction;
import us.ihmc.robotics.optimization.RealDomainBounds;
import us.ihmc.robotics.optimization.constrainedOptimization.ConstraintFunction;

import java.util.List;

public interface EvaluationProblem
{
   /**
    *
    * @return list of bounds, or null if not bounded
    */
   public RealDomainBounds[] getDomain();

   public CostFunction getCostFunction();

   public List<ConstraintFunction> getEqualityConstraints();
   public List<ConstraintFunction> getInequalityConstraints();
   public DMatrixD1 getOptimumParameters();
   public double getOptimumCost();
}
