package us.ihmc.quadrupedRobotics.util;

import java.util.HashMap;
import java.util.Map;

/**
 * THIS IS A HACK NOT
 * 
 * @author Kyle Cesare
 */
public class HeterogeneousMemoryPool
{
   private final Map<Class<?>, HomogeneousMemoryPool<?>> pools = new HashMap<>();

   @SuppressWarnings("unchecked")
   public <T> T grab(Class<T> type)
   {
      if (pools.containsKey(type))
      {
         return (T) pools.get(type).grab();
      }

      HomogeneousMemoryPool<T> pool = new HomogeneousMemoryPool<>(type);
      pools.put(type, pool);

      return (T) pool.grab();
   }

   public void reset()
   {
      for (HomogeneousMemoryPool<?> pool : pools.values())
      {
         pool.reset();
      }
   }
}
