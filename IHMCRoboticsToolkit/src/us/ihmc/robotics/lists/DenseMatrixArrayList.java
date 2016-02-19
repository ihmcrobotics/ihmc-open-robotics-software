package us.ihmc.robotics.lists;

import org.ejml.data.DenseMatrix64F;

public class DenseMatrixArrayList extends RecyclingArrayList<DenseMatrix64F>
{

   public DenseMatrixArrayList()
   {
      super(createBuilder());
   }

   public DenseMatrixArrayList(int initialCapacity)
   {
      super(initialCapacity, createBuilder());
   }

   public void set(DenseMatrixArrayList denseMatrixArrayList)
   {
      clear();
      for (int i = 0; i < denseMatrixArrayList.size(); i++)
         add().set(denseMatrixArrayList.get(i));
   }

   private static Builder<DenseMatrix64F> createBuilder()
   {
      Builder<DenseMatrix64F> builder = new Builder<DenseMatrix64F>()
      {
         @Override
         public DenseMatrix64F newInstance()
         {
            DenseMatrix64F denseMatrix64F = new DenseMatrix64F(1);
            denseMatrix64F.reshape(0, 0);
            return denseMatrix64F;
         }
      };
      return builder;
   }
}
