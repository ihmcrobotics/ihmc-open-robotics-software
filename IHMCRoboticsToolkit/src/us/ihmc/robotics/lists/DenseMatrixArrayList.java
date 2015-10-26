package us.ihmc.robotics.lists;

import org.ejml.data.DenseMatrix64F;

public class DenseMatrixArrayList extends RecyclingArrayList<DenseMatrix64F>
{

   public DenseMatrixArrayList()
   {
      super(DenseMatrix64F.class);
   }

   public DenseMatrixArrayList(int initialSize)
   {
      super(initialSize, DenseMatrix64F.class);
   }

   @Override
   protected DenseMatrix64F newInstance()
   {
      DenseMatrix64F denseMatrix64F = new DenseMatrix64F(1);
      denseMatrix64F.reshape(0, 0);
      return denseMatrix64F;
   }
}
