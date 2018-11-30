package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;

import java.util.function.Supplier;

public class VariableVectorBuilder implements Supplier<DenseMatrix64F>
{
   private final int rows;
   private final int cols;

   public VariableVectorBuilder(int rows, int cols)
   {
      this.rows = rows;
      this.cols = cols;
   }

   @Override
   public DenseMatrix64F get()
   {
      return new DenseMatrix64F(rows, cols);
   }
}
