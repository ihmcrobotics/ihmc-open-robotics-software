package us.ihmc.robotics.robotSide;

import java.util.EnumMap;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * A map of values by {@link RobotSide}.
 */
public class SideMap<V> extends EnumMap<RobotSide, V>
{
   private final transient RobotSide[][] sideArrays = new RobotSide[3][];
   {
      sideArrays[0] = new RobotSide[0];
      sideArrays[1] = new RobotSide[1];
      sideArrays[2] = RobotSide.values;
   }

   /**
    * Does not put values in the underlying EnumMap.
    */
   public SideMap()
   {
      super(RobotSide.class);
   }

   /**
    * Initializes both sides to the given values.
    */
   public SideMap(V leftValue, V rightValue)
   {
      super(RobotSide.class);
      set(leftValue, rightValue);
   }

   /**
    * Initializes both sides to values from the supplier.
    */
   public SideMap(Supplier<V> valueSupplier)
   {
      super(RobotSide.class);
      set(valueSupplier.get(), valueSupplier.get());
   }

   /**
    * Initializes both sides to values from the valueFunction.
    */
   public SideMap(Function<RobotSide, V> valueFunction)
   {
      super(RobotSide.class);
      set(valueFunction.apply(RobotSide.LEFT), valueFunction.apply(RobotSide.RIGHT));
   }

   /**
    * Copy constructor. Just copies the references to the objects; not a deep copy.
    * @param other the SideDependentList to be copied
    */
   public SideMap(Map<RobotSide, ? extends V> other)
   {
      super(RobotSide.class);

      for (RobotSide robotSide : RobotSide.values)
         put(robotSide, other.get(robotSide));
   }

   public void set(Map<RobotSide, ? extends V> sideDependentList)
   {
      for (RobotSide robotSide : RobotSide.values)
         put(robotSide, sideDependentList.get(robotSide));
   }

   public void set(Function<RobotSide, V> valueFunction)
   {
      for(RobotSide robotSide : RobotSide.values)
         put(robotSide, valueFunction.apply(robotSide));
   }

   public void set(V leftValue, V rightValue)
   {
      put(RobotSide.LEFT, leftValue);
      put(RobotSide.RIGHT, rightValue);
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

   @Override
   public String toString()
   {
      return "type: " + this.getClass() + "\n" + "left: " + get(RobotSide.LEFT) + "\n" + "right: " + get(RobotSide.RIGHT);
   }
}
