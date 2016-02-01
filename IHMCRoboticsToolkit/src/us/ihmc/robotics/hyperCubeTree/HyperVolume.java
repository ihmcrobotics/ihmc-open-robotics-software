package us.ihmc.robotics.hyperCubeTree;

public abstract class HyperVolume
{
   private final int dimensionality;
   private final OneDimensionalBounds[] outerBounds;
   private final OneDimensionalBounds[] innerBounds;
   private final boolean hasOuterBounds;
   private final boolean hasInnerBounds;

   public HyperVolume(int dimensionality, OneDimensionalBounds[] outerBounds, OneDimensionalBounds[] innerBounds, boolean hasFunction)
   {
      this.dimensionality = dimensionality;
      this.hasOuterBounds = outerBounds != null;
      this.outerBounds = outerBounds;
      this.hasInnerBounds = innerBounds != null;
      this.innerBounds = innerBounds;
      if (hasOuterBounds)
         testArrayDimensionality(outerBounds);
      if (hasInnerBounds)
         testArrayDimensionality(innerBounds);
   }

   public boolean outerBoundsIntersect(OneDimensionalBounds[] bounds)
   {
      if (hasOuterBounds)
      {
         testArrayDimensionality(bounds);
         for (int i = 0; i < dimensionality; i++)
         {
            if (!bounds[i].intersects(outerBounds[i]))
               return false;
         }
      }
      return true;

   }

   public boolean includedWithinInnerBounds(OneDimensionalBounds[] bounds)
   {
      if (hasInnerBounds)
      {
         OneDimensionalBounds[] innerBounds2 = this.innerBounds;
         int dimensionality2 = this.dimensionality;
         testArrayDimensionality(bounds);
         for (int i = 0; i < dimensionality2; i++)
         {
            if (!innerBounds2[i].isSupersetOf(bounds[i]))
               return false;
         }
         return true;
      }
      return false;
   }

   private void testArrayDimensionality(OneDimensionalBounds[] bounds)
   {
      if (bounds.length != dimensionality)
         throw new DimensionalityMismatchException();
   }

   protected abstract boolean complexBoundsIntersect(OneDimensionalBounds[] bounds);
   protected abstract double[] pointWithin(OneDimensionalBounds[] bounds);

   protected abstract boolean containsBoundsIfWithinOuterBounds(OneDimensionalBounds[] bounds);

   protected abstract boolean containsPointIfWithinOuterBounds(double[] point);

   public boolean intersectsBounds(OneDimensionalBounds[] bounds)
   {
      if (this.outerBoundsIntersect(bounds))
      {
         return this.complexBoundsIntersect(bounds);
      }
      return false;
   }
   
   public double[] intersectionWithBounds(OneDimensionalBounds[] bounds)
   {
      OneDimensionalBounds[] reducedBounds = OneDimensionalBounds.intersection(outerBounds, bounds);
      if (null==reducedBounds)
         return null;
      return pointWithin(reducedBounds);
   }

}
