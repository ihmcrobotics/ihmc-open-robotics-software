package us.ihmc.robotics.optimization;

import org.ejml.data.DMatrixD1;

public interface CostFunction
{
   public double calculate(DMatrixD1 x);

   public static CostFunction getEmptyCostFunction()
   {
      return new CostFunction()
      {
         @Override
         public double calculate(DMatrixD1 x)
         {
            return 0;
         }
      };
   }
}
