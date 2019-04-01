package us.ihmc.robotics.robotSide;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;

public class SideDependentList<V> extends SegmentDependentList<RobotSide, V> implements Iterable<V>
{
   private final transient RobotSide[][] sideArrays = new RobotSide[3][];
   {
      sideArrays[0] = new RobotSide[0];
      sideArrays[1] = new RobotSide[1];
      sideArrays[2] = RobotSide.values;
   }

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
   public SideDependentList(SegmentDependentList<RobotSide, ? extends V> other)
   {
      super(RobotSide.class);

      for (RobotSide robotSide : RobotSide.values)
      {
         this.set(robotSide, other.get(robotSide));
      }
   }

   public void set(SideDependentList<V> sideDependentList)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         this.set(robotSide, sideDependentList.get(robotSide));
      }
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
      return new String("type: " + this.getClass() + "\n" + "left: " + get(RobotSide.LEFT) + "\n" + "right: " + get(RobotSide.RIGHT));
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

   @Override
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

      @Override
      public boolean hasNext()
      {
         return state < 2;
      }

      @Override
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

      @Override
      public void remove()
      {
         throw new UnsupportedOperationException("Cannot remove elements from a SideDependentList.");
      }
   }
}
