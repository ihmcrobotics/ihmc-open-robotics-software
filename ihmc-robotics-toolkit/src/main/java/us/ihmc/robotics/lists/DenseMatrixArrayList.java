package us.ihmc.robotics.lists;

import org.ejml.data.DenseMatrix64F;

import java.util.function.Supplier;

public class DenseMatrixArrayList extends RecyclingArrayList<DenseMatrix64F>
{

   public DenseMatrixArrayList()
   {
      this(0);
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

   private static Supplier<DenseMatrix64F> createBuilder()
   {
      return () ->
      {
         DenseMatrix64F denseMatrix64F = new DenseMatrix64F(1);
         denseMatrix64F.reshape(0, 0);
         return denseMatrix64F;
      };
   }
}
