package us.ihmc.robotics.quadTree;

import java.util.LinkedHashMap;
import java.util.Map;

public class QuadTreeForGroundPointLimiter extends LinkedHashMap<QuadTreeForGroundPoint, Object>
{
   private static final long serialVersionUID = 5582769749269559171L;
   private final static Object PRESENT = new Object();
   private final int maximumSize;
   
   public QuadTreeForGroundPointLimiter(int maximumSize)
   {
      super(maximumSize + 1, 1.1f, false);   // Never trigger resize,
      this.maximumSize = maximumSize;
   }
   
   public void add(QuadTreeForGroundPoint point)
   {
      if(!point.isRegistered())
      {
         point.setRegistered(true);
         put(point, PRESENT);
      }
   }
   
   public Object remove(Object point)
   {
      return super.remove(point);
   }
   
   @Override
   public boolean removeEldestEntry(Map.Entry<QuadTreeForGroundPoint, Object> eldest)
   {
      if(size() > maximumSize)
      {
         QuadTreeForGroundPoint key = eldest.getKey();
         key.removeFromParent();
         return true;
      }
      else
      {
         return false;
      }
   }
   
}
