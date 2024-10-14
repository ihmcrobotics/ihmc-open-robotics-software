package us.ihmc.robotics.lists;

import java.util.List;
import java.util.function.Supplier;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;

import us.ihmc.commons.lists.RecyclingArrayList;

public class DenseMatrixArrayList extends RecyclingArrayList<DMatrixRMaj>
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

   private static Supplier<DMatrixRMaj> createBuilder()
   {
      return () -> {
         DMatrixRMaj denseMatrix64F = new DMatrixRMaj(1);
         denseMatrix64F.reshape(0, 0);
         return denseMatrix64F;
      };
   }

   @SuppressWarnings("rawtypes")
   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof List)
      {
         List other = (List) object;

         if (size() != other.size())
            return false;

         for (int i = 0; i < size(); i++)
         {
            Object otherElement = other.get(i);
            if (otherElement instanceof DMatrixRMaj)
            {
               if (!MatrixFeatures_DDRM.isEquals(get(i), (DMatrixRMaj) otherElement))
                  return false;
            }
            else
            {
               return false;
            }
         }

         return true;
      }
      else
      {
         return false;
      }
   }
}
