package us.ihmc.robotics.robotSide;

import java.util.EnumMap;

public class SegmentDependentList<E extends Enum<E> & RobotSegment<E>, V> extends EnumMap<E, V>
{ 

   public SegmentDependentList(Class<E> keyClass)
   {
      super(keyClass);
   }
   
   public V get(E key)
   {
      return super.get(key);
   }

   public V set(E segment, V element)
   {
      return this.put(segment, element);
   }


   public boolean containsSegment(E robotSegment)
   {
      return this.containsKey(robotSegment);
   }
   
}
