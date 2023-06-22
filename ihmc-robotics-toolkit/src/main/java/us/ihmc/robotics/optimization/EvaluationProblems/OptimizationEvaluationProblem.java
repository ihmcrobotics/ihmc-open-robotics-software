package us.ihmc.robotics.optimization.EvaluationProblems;

import org.ejml.data.DMatrixD1;
import us.ihmc.robotics.optimization.OptimizationProblem;

public interface OptimizationEvaluationProblem extends OptimizationProblem
{
   public DMatrixD1 getOptimumParameters();
   public double getOptimumCost();
}
