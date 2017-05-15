package us.ihmc.robotics.robotSide;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;

public class SideDependentList<V> extends EnumMap<RobotSide, V> implements Iterable<V>
{
   private static final long serialVersionUID = -6514328471068877058L;

   public SideDependentList()
   {
      super(RobotSide.class);
   }

   public SideDependentList(V leftObject, V rightObject)
   {
      super(RobotSide.class);
      this.put(RobotSide.LEFT, leftObject);
      this.put(RobotSide.RIGHT, rightObject);
   }

   /**
    * Copy constructor. Just copies the references to the objects; not a deep copy.
    * @param other the SideDependentList to be copied
    */
   public SideDependentList(SideDependentList<? extends V> other)
   {
      super(RobotSide.class);

      for (RobotSide robotSide : RobotSide.values)
      {
         this.set(robotSide, other.get(robotSide));
      }
   }
   
   public V get(RobotSide side)
   {
      return super.get(side);
   }

   public String toString()
   {
      return new String("type: " + this.getClass() + "\n" + "left: " + get(RobotSide.LEFT) + "\n" + "right: " + get(RobotSide.RIGHT));
   }

   public V set(RobotSide robotSide, V element)
   {
      return this.put(robotSide, element);
   }

   public void set(SideDependentList<V> sideDependentList)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         this.set(robotSide, sideDependentList.get(robotSide));
      }
   }

   public static <K extends Enum<K>, V> SideDependentList<EnumMap<K, V>> createListOfEnumMaps(Class<K> keyType)
   {
      return new SideDependentList<EnumMap<K, V>>(new EnumMap<K, V>(keyType), new EnumMap<K, V>(keyType));
   }

   public static <K, V> SideDependentList<Map<K, V>> createListOfHashMaps()
   {
      return new SideDependentList<Map<K, V>>(new LinkedHashMap<K, V>(), new LinkedHashMap<K, V>());
   }
   
   public static <V> SideDependentList<ArrayList<V>> createListOfArrayLists()
   {
      return new SideDependentList<ArrayList<V>>(new ArrayList<V>(), new ArrayList<V>());
   }

   public Iterator<V> iterator()
   {
      return new Itr();
   }



   private class Itr implements Iterator<V>
   {
      private int state;

      public Itr()
      {
         this.state = 0;
      }

      public boolean hasNext()
      {
         return state < 2;
      }

      public V next()
      {
         if (state == 0)
         {
            state++;

            return get(RobotSide.LEFT);
         }
         else if (state == 1)
         {
            state++;

            return get(RobotSide.RIGHT);
         }
         else
         {
            throw new IndexOutOfBoundsException();
         }
      }

      public void remove()
      {
         throw new UnsupportedOperationException("Cannot remove elements from a SideDependentList.");
      }
   }
}
