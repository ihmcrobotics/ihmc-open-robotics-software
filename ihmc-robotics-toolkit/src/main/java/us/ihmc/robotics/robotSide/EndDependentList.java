package us.ihmc.robotics.robotSide;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;

public class EndDependentList<V> extends EnumMap<RobotEnd, V> implements Iterable<V>
{
   private static final long serialVersionUID = -6514328471068877058L;

   public EndDependentList()
   {
      super(RobotEnd.class);
   }

   public EndDependentList(V hindObject, V frontObject)
   {
      super(RobotEnd.class);
      this.put(RobotEnd.HIND, hindObject);
      this.put(RobotEnd.FRONT, frontObject);
   }

   /**
    * Copy constructor. Just copies the references to the objects; not a deep copy.
    * @param other the EndDependentList to be copied
    */
   public EndDependentList(EndDependentList<V> other)
   {
      super(RobotEnd.class);

      for (RobotEnd robotEnd : RobotEnd.values)
      {
         this.set(robotEnd, other.get(robotEnd));
      }
   }

   public V get(RobotEnd robotEnd)
   {
      return super.get(robotEnd);
   }

   public String toString()
   {
      return new String("type: " + this.getClass() + "\n" + "hind: " + get(RobotEnd.HIND) + "\n" + "front: " + get(RobotEnd.FRONT));
   }

   public V set(RobotEnd robotEnd, V element)
   {
      return this.put(robotEnd, element);
   }

   public void set(EndDependentList<V> endDependentList)
   {
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         this.set(robotEnd, endDependentList.get(robotEnd));
      }
   }

   public static <K extends Enum<K>, V> EndDependentList<EnumMap<K, V>> createListOfEnumMaps(Class<K> keyType)
   {
      return new EndDependentList<EnumMap<K, V>>(new EnumMap<K, V>(keyType), new EnumMap<K, V>(keyType));
   }

   public static <K, V> EndDependentList<Map<K, V>> createListOfHashMaps()
   {
      return new EndDependentList<Map<K, V>>(new LinkedHashMap<K, V>(), new LinkedHashMap<K, V>());
   }

   public static <V> EndDependentList<ArrayList<V>> createListOfArrayLists()
   {
      return new EndDependentList<ArrayList<V>>(new ArrayList<V>(), new ArrayList<V>());
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

            return get(RobotEnd.HIND);
         }
         else if (state == 1)
         {
            state++;

            return get(RobotEnd.FRONT);
         }
         else
         {
            throw new IndexOutOfBoundsException();
         }
      }

      public void remove()
      {
         throw new UnsupportedOperationException("Cannot remove elements from a EndDependentList.");
      }
   }
}
