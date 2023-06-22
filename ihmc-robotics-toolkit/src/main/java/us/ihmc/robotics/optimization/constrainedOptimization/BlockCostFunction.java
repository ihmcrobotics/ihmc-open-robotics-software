package us.ihmc.robotics.optimization.constrainedOptimization;

import org.ejml.data.DMatrixD1;

public interface BlockCostFunction
{
   public double calculate(DMatrixD1[] blocks);

   public static BlockCostFunction getEmptyCostFunction()
   {
      return new BlockCostFunction()
      {
         @Override
         public double calculate(DMatrixD1[] blocks)
         {
            return 0;
         }
      };
   }
}
