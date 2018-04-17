package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class DiscreteSequence extends RecyclingArrayList<DenseMatrix64F> implements DiscreteData
{
   public DiscreteSequence(int dimensionality)
   {
      this(dimensionality, 1);
   }

   public DiscreteSequence(int xDimension, int yDimension)
   {
      super(1000, new VariableVectorBuilder(xDimension, yDimension));
      this.clear();
   }

   public void set(DiscreteData other)
   {
      this.clear();
      for (int i = 0; i < other.size(); i++)
         this.add().set(other.get(i));
   }

   public void setLength(int length)
   {
      this.clear();
      for (int i = 0; i < length; i++)
         this.add().zero();
   }

   @Override
   public void setZero(int size)
   {
      this.clear();
      for (int i = 0; i < size; i++)
         this.add().zero();
   }
}
