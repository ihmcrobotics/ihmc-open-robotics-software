package us.ihmc.robotics.hyperCubeTree;

public class HyperCubeLeaf<T>
{
   private final T value;
   private final double[] location;

   public HyperCubeLeaf(T value, double[] location)
   {
      this.value = value;
      this.location = location;
   }

   public T getValue()
   {
      return value;
   }

   public double[] getLocation()
   {
      return location;
   }

}