package us.ihmc.commons.robotics.robotSide;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;

public class QuadrantDependentList<V> extends SegmentDependentList<RobotQuadrant, V>
{
   private final RobotQuadrant[][] quadrantArrays = new RobotQuadrant[5][];
   {
      quadrantArrays[0] = new RobotQuadrant[0];
      quadrantArrays[1] = new RobotQuadrant[1];
      quadrantArrays[2] = new RobotQuadrant[2];
      quadrantArrays[3] = new RobotQuadrant[3];
      quadrantArrays[4] = RobotQuadrant.values;
   }

   public QuadrantDependentList()
   {
      super(RobotQuadrant.class);
   }

   public QuadrantDependentList(Supplier<V> allocator)
   {
      this(allocator.get(), allocator.get(), allocator.get(), allocator.get());
   }
   
   public QuadrantDependentList(V frontLeftObject, V frontRightObject, V hindLeftObject, V hindRightObject)
   {
      super(RobotQuadrant.class);
      set(RobotQuadrant.FRONT_LEFT, frontLeftObject);
      set(RobotQuadrant.FRONT_RIGHT, frontRightObject);
      set(RobotQuadrant.HIND_RIGHT, hindRightObject);
      set(RobotQuadrant.HIND_LEFT, hindLeftObject);
   }

   /**
    * Copy constructor. Just copies the references to the objects; not a deep copy.
    * @param other the QuadrantDependentList to be copied
    */
   public QuadrantDependentList(SegmentDependentList<RobotQuadrant, ? extends V> other)
   {
      super(RobotQuadrant.class);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         this.set(robotQuadrant, other.get(robotQuadrant));
      }
   }

   public RobotQuadrant[] quadrants()
   {
      fillQuadrantArray();
      return quadrantArrays[size()];
   }

   private void fillQuadrantArray()
   {
      if (size() == 4)
         return;

      for (int i = 0, j = 0; i < RobotQuadrant.values.length; i++)
      {
         if (containsKey(RobotQuadrant.values[i]))
         {
            quadrantArrays[size()][j++] = RobotQuadrant.values[i];
         }
      }
   }


   public static <K extends Enum<K>, V> QuadrantDependentList<EnumMap<K, V>> createListOfEnumMaps(Class<K> keyType)
   {
      return new QuadrantDependentList<EnumMap<K, V>>(new EnumMap<K, V>(keyType), new EnumMap<K, V>(keyType),new EnumMap<K, V>(keyType),new EnumMap<K, V>(keyType));
   }

   public static <K, V> QuadrantDependentList<Map<K, V>> createListOfHashMaps()
   {
      return new QuadrantDependentList<Map<K, V>>(new LinkedHashMap<K, V>(), new LinkedHashMap<K, V>(),new LinkedHashMap<K, V>(),new LinkedHashMap<K, V>());
   }
   
   public static <V> QuadrantDependentList<ArrayList<V>> createListOfArrayLists()
   {
      return new QuadrantDependentList<ArrayList<V>>(new ArrayList<V>(), new ArrayList<V>(), new ArrayList<V>(), new ArrayList<V>());
   }

   public static <V> QuadrantDependentList<V> build(Function<RobotQuadrant, V> values)
   {
      QuadrantDependentList<V> quadrantDependentList = new QuadrantDependentList<>();

      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         quadrantDependentList.put(quadrant, values.apply(quadrant));
      }

      return quadrantDependentList;
   }
}
