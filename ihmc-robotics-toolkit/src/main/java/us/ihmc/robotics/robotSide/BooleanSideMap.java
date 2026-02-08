package us.ihmc.robotics.robotSide;

import java.util.function.BooleanSupplier;
import java.util.function.Predicate;

/**
 * A map of boolean values by {@link RobotSide}.
 *
 * TODO: Unfinished, will probably delete
 */
public class BooleanSideMap
{
   private final transient RobotSide[][] sideArrays = new RobotSide[3][];
   {
      sideArrays[0] = new RobotSide[0];
      sideArrays[1] = new RobotSide[1];
      sideArrays[2] = RobotSide.values;
   }

   private boolean leftValue;
   private boolean rightValue;

   public BooleanSideMap()
   {
   }

   public BooleanSideMap(boolean leftValue, boolean rightValue)
   {
      set(leftValue, rightValue);
   }

   public BooleanSideMap(BooleanSupplier valueSupplier)
   {
      set(valueSupplier.getAsBoolean(), valueSupplier.getAsBoolean());
   }

   public BooleanSideMap(Predicate<RobotSide> valueFunction)
   {
      set(valueFunction.test(RobotSide.LEFT), valueFunction.test(RobotSide.RIGHT));
   }

   public BooleanSideMap(BooleanSideMap other)
   {
      set(other.leftValue, other.rightValue);
   }

   public boolean get(RobotSide robotSide)
   {
      if (robotSide == null)
         throw new NullPointerException("robotSide");

      return robotSide == RobotSide.LEFT ? leftValue : rightValue;
   }

   public boolean getLeft()
   {
      return leftValue;
   }

   public boolean getRight()
   {
      return rightValue;
   }

   public void set(RobotSide robotSide, boolean value)
   {
      if (robotSide == null)
         throw new NullPointerException("robotSide");

      if (robotSide == RobotSide.LEFT)
         leftValue = value;
      else
         rightValue = value;
   }

   public void set(boolean leftValue, boolean rightValue)
   {
      this.leftValue = leftValue;
      this.rightValue = rightValue;
   }

   public void set(Predicate<RobotSide> valueFunction)
   {
      set(valueFunction.test(RobotSide.LEFT), valueFunction.test(RobotSide.RIGHT));
   }

   public void set(BooleanSideMap other)
   {
      set(other.leftValue, other.rightValue);
   }

   public RobotSide[] sides()
   {
      fillSideArray();
      return sideArrays[size()];
   }

   private void fillSideArray()
   {
      if (size() == 2)
         return;

      for (int i = 0, j = 0; i < RobotSide.values.length; i++)
      {
         if (containsKey(RobotSide.values[i]))
         {
            sideArrays[size()][j++] = RobotSide.values[i];
         }
      }
   }

   public int size()
   {
      return leftValue ? (rightValue ? 2 : 1) : (rightValue ? 1 : 0);
   }

   public boolean containsKey(RobotSide robotSide)
   {
      // TODO
      return false;
   }

   @Override
   public String toString()
   {
      return "type: " + this.getClass() + "\n" + "left: " + leftValue + "\n" + "right: " + rightValue;
   }
}
