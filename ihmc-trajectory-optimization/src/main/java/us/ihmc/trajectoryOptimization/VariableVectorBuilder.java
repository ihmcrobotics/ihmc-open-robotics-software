package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.lists.GenericTypeBuilder;

public class VariableVectorBuilder extends GenericTypeBuilder<DenseMatrix64F>
{
   private final int rows;
   private final int cols;

   public VariableVectorBuilder(int rows, int cols)
   {
      this.rows = rows;
      this.cols = cols;
   }

   @Override
   public DenseMatrix64F newInstance()
   {
      return new DenseMatrix64F(rows, cols);
   }
}
