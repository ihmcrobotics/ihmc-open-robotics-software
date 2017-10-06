package us.ihmc.robotics.hyperCubeTree;

public class OneDimensionalBounds
{
   double midpoint = Double.NaN;
   final double min;
   final double max;

   public OneDimensionalBounds(Double bound1, Double bound2)
   {
      this.min = (bound2 >= bound1)?bound1:bound2;
      this.max = (bound2 >= bound1)?bound2:bound1;
      if (Double.isNaN(bound1) || Double.isNaN(bound2))
      {
         throw new RuntimeException("illegal Bounds " + bound1 + " and " + bound2);
      }
   }

   public double max()
   {
      return max;
   }

   public double min()
   {
      return min;
   }

   public double midpoint()
   {
      if (isInfinite())
      {
         throw new RuntimeException("It is impossible to bisect an infinite range.");
      }
      if (Double.isNaN(midpoint))
      {
         this.midpoint = (min() + max()) * 0.5;
      }
      return midpoint;
   }

   public double size()
   {
      return max - min;
   }

   public boolean contains(double testValue)
   {
      return (testValue <= max && testValue >= min);
   }

   public boolean maxSide(double testValue)
   {
      return (testValue >= midpoint());
   }

   public OneDimensionalBounds subdivide(boolean maxSide)
   {
      if (maxSide)
         return new OneDimensionalBounds(midpoint(), max);
      return new OneDimensionalBounds(min, midpoint());
   }

   public boolean intersects(OneDimensionalBounds other)
   {
      return other.min <= this.max && this.min <= other.max;
   }

   public boolean isSupersetOf(OneDimensionalBounds other)
   {
      return other.min >= this.min && this.max >= other.max;
   }

   public boolean isSubsetOf(OneDimensionalBounds other)
   {
      return other.min <= this.min && this.max <= other.max;
   }

   public OneDimensionalBounds intersectionWith(OneDimensionalBounds other)
   {
      if (intersects(other))
      {
         return new OneDimensionalBounds(Math.max(this.min(), other.min()), Math.min(this.max(), other.max()));
      }
      return null;
   }

   public static OneDimensionalBounds unbounded()
   {
      return new OneDimensionalBounds(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
   }

   public boolean isInfinite()
   {
      return (Double.isInfinite(max) || Double.isInfinite(min));
   }
   
   public double scale (double valueBetweenZeroAndOne)
   {
      return min()+size()*valueBetweenZeroAndOne;
   }
   public double unScale(double value)
   {
      return (value-min())/size();
   }

   public String toString()
   {
      if (isInfinite())
      {
         return "infinite range " + (Double.isInfinite(min) ? "(" : "[") + min + ", " + max + (Double.isInfinite(max) ? ")" : "]");
      }
      return "finite, inclusive range [" + min + ", " + max + "]";
   }

   public static OneDimensionalBounds[] intersection(OneDimensionalBounds[] boundsA, OneDimensionalBounds[] boundsB)
   {
      if (boundsB.length != boundsA.length)
         throw new DimensionalityMismatchException();
      OneDimensionalBounds[] intersection = new OneDimensionalBounds[boundsA.length];
      for (int i = 0; i < boundsA.length; i++)
      {
         intersection[i]=boundsA[i].intersectionWith(boundsB[i]);
         if (null==intersection[i])
         {
            return null;
         }
      }
      return intersection;
   }
}