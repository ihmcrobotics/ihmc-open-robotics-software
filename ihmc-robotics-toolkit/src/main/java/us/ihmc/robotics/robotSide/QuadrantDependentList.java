package us.ihmc.robotics.robotSide;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.Map;

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
   
   public QuadrantDependentList(V frontLeftObject, V frontRightObject, V hindLeftObject, V hindRightObject)
   {
      super(RobotQuadrant.class);
      set(RobotQuadrant.FRONT_LEFT, frontLeftObject);
      set(RobotQuadrant.FRONT_RIGHT, frontRightObject);
      set(RobotQuadrant.HIND_RIGHT, hindRightObject);
      set(RobotQuadrant.HIND_LEFT, hindLeftObject);
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
}
