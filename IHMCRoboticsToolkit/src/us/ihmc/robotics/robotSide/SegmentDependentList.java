package us.ihmc.robotics.robotSide;

import java.util.EnumMap;

public class SegmentDependentList<E extends Enum<E>, V>
{ 
   private final EnumMap<E, V> map;
   public E[] values;
   
   public SegmentDependentList(Class<E> keyClass)
   {
      map = new EnumMap<E, V>(keyClass);
      values = keyClass.getEnumConstants();
   }
   
   public V get(RobotSegment<E> key)
   {
      return map.get(key);
   }
   
   public V get(E key)
   {
      return map.get(key);
   }
   
   public void set(E robotSegment, V element)
   {
      map.put(robotSegment, element);
   }
   
   public V remove(E robotSegment)
   {
      V element = map.remove(robotSegment);
      return element;
   }
   
   public void clear()
   {
      map.clear();
   }
   
   
   public boolean containsSegment(E robotSegment)
   {
      return map.containsKey(robotSegment);
   }
   
   public int size()
   {
      return map.size();
   }
   
   public static void main(String[] args)
   {
      SegmentDependentList<RobotQuadrant, String> segmentDependentList = new SegmentDependentList<>(RobotQuadrant.class);
      segmentDependentList.set(RobotQuadrant.FRONT_LEFT, "FRONTLEFT");
      segmentDependentList.set(RobotQuadrant.FRONT_RIGHT, "FRONTRIGHT");
      segmentDependentList.set(RobotQuadrant.HIND_LEFT, "HINDLEFT");
      segmentDependentList.set(RobotQuadrant.HIND_RIGHT, "HINDRIGHT");
      
      for(RobotQuadrant robotQuadrant : segmentDependentList.values)
      {
         System.out.println(robotQuadrant);
      }
      
      for(RobotQuadrant robotQuadrant : segmentDependentList.values)
      {
         System.out.println(segmentDependentList.get(robotQuadrant));
      }
   }
}
