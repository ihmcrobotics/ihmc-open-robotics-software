package us.ihmc.trajectoryOptimization;

import java.util.function.Supplier;

import org.ejml.data.DMatrixRMaj;

public class VariableVectorBuilder implements Supplier<DMatrixRMaj>
{
   private final int rows;
   private final int cols;

   public VariableVectorBuilder(int rows, int cols)
   {
      this.rows = rows;
      this.cols = cols;
   }

   @Override
   public DMatrixRMaj get()
   {
      return new DMatrixRMaj(rows, cols);
   }
}
