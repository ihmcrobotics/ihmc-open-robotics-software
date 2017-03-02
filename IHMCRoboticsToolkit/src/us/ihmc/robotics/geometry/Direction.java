package us.ihmc.robotics.geometry;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

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

   public static double get(Tuple3DBasics tuple, Direction direction)
   {
      switch (direction)
      {
         case X :
            return tuple.getX();
   
         case Y :
            return tuple.getY();
   
         case Z :
            return tuple.getZ();
   
         default :
            throw new IndexOutOfBoundsException();
      }
   }

   public static void set(Tuple3DBasics tuple, Direction direction, double value)
   {
      switch (direction)
      {
         case X :
            tuple.setX(value);
   
            break;
   
         case Y :
            tuple.setY(value);
   
            break;
   
         case Z :
            tuple.setZ(value);
   
            break;
   
         default :
            throw new IndexOutOfBoundsException();
      }
   }
}
