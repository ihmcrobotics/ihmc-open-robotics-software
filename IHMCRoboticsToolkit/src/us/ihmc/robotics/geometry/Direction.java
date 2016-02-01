package us.ihmc.robotics.geometry;

public enum Direction
{
   X(0), Y(1), Z(2);
   
   public static final Direction[] values = values();
   
   private static final Direction[] values2D = {X, Y};
   
   public static Direction[] values2D()
   {
      return values2D;
   }

   private final int index;
   
   private Direction(int index)
   {
      this.index = index;
   }

   public int getIndex()
   {
      return index;
   }
}
