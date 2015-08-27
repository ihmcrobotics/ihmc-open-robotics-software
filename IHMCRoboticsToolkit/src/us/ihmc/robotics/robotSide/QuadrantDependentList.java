package us.ihmc.robotics.robotSide;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;

@SuppressWarnings("serial")
public class QuadrantDependentList<V> extends LinkedHashMap<RobotQuadrant, V> implements Iterable<V>
{
   public QuadrantDependentList()
   {
      super();
   }
   
   public QuadrantDependentList(V frontLeftObject, V frontRightObject, V backLeftObject, V backRightObject)
   {
      super();
      this.put(RobotQuadrant.FRONT_LEFT, frontLeftObject);
      this.put(RobotQuadrant.FRONT_RIGHT, frontRightObject);
      this.put(RobotQuadrant.HIND_LEFT, backLeftObject);
      this.put(RobotQuadrant.HIND_RIGHT, backRightObject);
   }
   
   public V get(RobotQuadrant key)
   {
      return super.get(key);
   }
   
   public V set(RobotQuadrant robotQuadrant, V element)
   {
      return this.put(robotQuadrant, element);
   }
   
   @Override
   public Iterator<V> iterator()
   {
      return new MyIterator();
   }
   
   private class MyIterator implements Iterator<V>
   {
      private int state;

      public MyIterator()
      {
         this.state = 0;
      }
      
      @Override
      public boolean hasNext()
      {
         return state < 4;
      }

      @Override
      public V next()
      {
         if (state == 0)
         {
            ++state;
            return get(RobotQuadrant.getQuadrant(RobotEnd.HIND, RobotSide.LEFT));
         }
         else if (state == 1)
         {
            ++state;
            return get(RobotQuadrant.getQuadrant(RobotEnd.HIND, RobotSide.RIGHT));
         }
         else if (state == 2)
         {
            ++state;
            return get(RobotQuadrant.getQuadrant(RobotEnd.FRONT, RobotSide.LEFT));
         }
         else if (state == 3)
         {
            ++state;
            return get(RobotQuadrant.getQuadrant(RobotEnd.FRONT, RobotSide.RIGHT));
         }
         else
         {
            throw new IndexOutOfBoundsException();
         }
      }

      public void remove()
      {
         throw new UnsupportedOperationException("Cannot remove elements from a QuadrantDependentList.");
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
