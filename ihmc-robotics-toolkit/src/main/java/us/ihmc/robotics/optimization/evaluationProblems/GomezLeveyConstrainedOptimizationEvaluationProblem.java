package us.ihmc.robotics.optimization.evaluationProblems;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.robotics.optimization.CostFunction;
import us.ihmc.robotics.optimization.RealDomainBounds;
import us.ihmc.robotics.optimization.constrainedOptimization.ConstraintFunction;

import java.util.ArrayList;
import java.util.List;

public class GomezLeveyConstrainedOptimizationEvaluationProblem implements OptimizationEvaluationProblem
{
   @Override
   public RealDomainBounds[] getDomain()
   {
      return new RealDomainBounds[]{
            new RealDomainBounds(-1.0, 0.75),
            new RealDomainBounds(-1.0, 1.0)
      };
   }

   @Override
   public CostFunction getCostFunction()
   {
      CostFunction function = new CostFunction()
      {
         @Override
         public double calculate(DMatrixD1 x)
         {
            double x0 = x.get(0);
            double x1 = x.get(1);
            return 4.0 * x0 * x0 - 2.1 * Math.pow(x0, 4) + Math.pow(x0, 6) / 3.0 + x0 * x1 - 4 * x1 * x1 + 4 * Math.pow(x1, 4);
         }
      };
      return function;
   }

   @Override
   public List<ConstraintFunction> getEqualityConstraints()
   {
      List<ConstraintFunction> constraints = new ArrayList<>();
      constraints.add(new ConstraintFunction()
      {
         @Override
         public double calculate(DMatrixD1 x)
         {
            double x0 = x.get(0);
            double x1 = x.get(1);
            return -(-Math.sin(4.0 * Math.PI * x0) + 2.0 * Math.pow(Math.sin(2 * Math.PI * x1), 2) - 1.5);
         }
      });
      return constraints;
   }

   @Override
   public List<ConstraintFunction> getInequalityConstraints()
   {
      return new ArrayList<>();
   }

   @Override
   public DMatrixD1 getOptimumParameters()
   {
      return new DMatrixRMaj(new double[]{0.08984201, -0.7126564});
   }

   @Override
   public double getOptimumCost()
   {
      return -1.031628453;
   }
}
