package us.ihmc.robotics.optimization.EvaluationProblems;

import org.ejml.data.DMatrixD1;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.mult.VectorVectorMult_DDRM;
import us.ihmc.robotics.optimization.CostFunction;
import us.ihmc.robotics.optimization.RealDomainBounds;
import us.ihmc.robotics.optimization.constrainedOptimization.ConstraintFunction;

import java.util.ArrayList;
import java.util.List;

public class AckleyOptimizationEvaluationProblem implements OptimizationEvaluationProblem
{
   private final double c;
   private final double a;
   private final double b;
   private final int parameterDimensions;
   private final DMatrixD1 optimumParameters;
   private final double optimumValue = 0;

   public AckleyOptimizationEvaluationProblem(int parameterDimensions)
   {
      this(20.0, 0.2, 2.0 * Math.PI, parameterDimensions);
   }

   public AckleyOptimizationEvaluationProblem(double a, double b, double c, int parameterDimensions)
   {
      this.a = a;
      this.b = b;
      this.c = c;
      this.parameterDimensions = parameterDimensions;
      optimumParameters = new DMatrixRMaj(parameterDimensions, 1);
   }

   @Override
   public RealDomainBounds[] getDomain()
   {
      RealDomainBounds[] bounds = new RealDomainBounds[parameterDimensions];
      for (int i = 0; i < parameterDimensions; i++)
      {
         bounds[i] = new RealDomainBounds(-32.768, 32.768);
      }
      return bounds;
   }

   @Override
   public CostFunction getCostFunction()
   {
      CostFunction function = new CostFunction()
      {
         @Override
         public double calculate(DMatrixD1 x)
         {
            double norm = VectorVectorMult_DDRM.innerProd(x, x);
            norm = Math.sqrt(norm);

            double cosineSummation = 0;
            for (int i = 0; i < parameterDimensions; i++)
            {
               cosineSummation += Math.cos(c * x.get(i));
            }

            return -a * Math.exp(-b / Math.sqrt(parameterDimensions) * norm) - Math.exp(cosineSummation/parameterDimensions) + a + Math.exp(1);
         }
      };
      return function;
   }

   @Override
   public List<ConstraintFunction> getEqualityConstraints()
   {
      return new ArrayList<>();
   }

   @Override
   public List<ConstraintFunction> getInequalityConstraints()
   {
      return new ArrayList<>();
   }

   @Override
   public DMatrixD1 getOptimumParameters()
   {
      return optimumParameters;
   }

   @Override
   public double getOptimumCost()
   {
      return optimumValue;
   }
}
