package us.ihmc.robotics.robotSide;

import java.util.Iterator;
import java.util.LinkedHashMap;

@SuppressWarnings("serial")
public class QuadrantDependentList<V> extends LinkedHashMap<RobotQuadrant, V> implements Iterable<V>
{
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
            return get(RobotQuadrant.getQuadrant(RobotEnd.BACK, RobotSide.LEFT));
         }
         else if (state == 1)
         {
            ++state;
            return get(RobotQuadrant.getQuadrant(RobotEnd.BACK, RobotSide.RIGHT));
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
}
