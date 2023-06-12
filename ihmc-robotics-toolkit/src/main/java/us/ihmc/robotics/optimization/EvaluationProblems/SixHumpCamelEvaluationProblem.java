package us.ihmc.robotics.optimization.EvaluationProblems;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.robotics.optimization.CostFunction;
import us.ihmc.robotics.optimization.RealDomainBounds;
import us.ihmc.robotics.optimization.constrainedOptimization.ConstraintFunction;

import java.util.List;

public class SixHumpCamelEvaluationProblem implements EvaluationProblem
{
   @Override
   public RealDomainBounds[] getDomain()
   {
      return new RealDomainBounds[]{
         new RealDomainBounds(-3.0, 3.0),
         new RealDomainBounds(-2.0, 2.0)
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
            return (4.0 - 2.1 * x0 * x0 + Math.pow(x0, 4) / 3.0) * x0 * x0 +
                   x0 * x1 +
                   (-4.0 + 4.0 * x1 * x1) * x1 * x1;
         }
      };
      return function;
   }

   @Override
   public List<ConstraintFunction> getEqualityConstraints()
   {
      return null;
   }

   @Override
   public List<ConstraintFunction> getInequalityConstraints()
   {
      return null;
   }

   @Override
   public DMatrixD1 getOptimumParameters()
   {
      return new DMatrixRMaj(new double[]{0.0898, -0.7126});
   }

   @Override
   public double getOptimumCost()
   {
      return -1.0316;
   }
}
