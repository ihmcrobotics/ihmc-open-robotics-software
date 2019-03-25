package us.ihmc.robotics.lists;

import java.util.List;
import java.util.function.Supplier;

import org.ejml.data.D1Matrix64F;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.MatrixFeatures;

import us.ihmc.commons.lists.RecyclingArrayList;

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
      return () -> {
         DenseMatrix64F denseMatrix64F = new DenseMatrix64F(1);
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
            if (otherElement instanceof D1Matrix64F)
            {
               if (!MatrixFeatures.isEquals(get(i), (D1Matrix64F) otherElement))
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
