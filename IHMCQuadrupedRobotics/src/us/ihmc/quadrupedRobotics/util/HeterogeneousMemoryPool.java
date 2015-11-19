package us.ihmc.quadrupedRobotics.util;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

public class HeterogeneousMemoryPool
{
   private final Map<Class<?>, HomogeneousMemoryPool<?>> pools = new HashMap<>();
   
   // Cache the values list to prevent allocation in the reset loop.
   private final Collection<HomogeneousMemoryPool<?>> values = pools.values();

   @SuppressWarnings("unchecked")
   public <T> T grab(Class<T> type)
   {
      if (pools.containsKey(type))
      {
         return (T) pools.get(type).grab();
      }

      HomogeneousMemoryPool<T> pool = new HomogeneousMemoryPool<>(type);

      pools.put(type, pool);
      values.add(pool);

      return (T) pool.grab();
   }

   public void reset()
   {
      
      for (HomogeneousMemoryPool<?> pool : values)
      {
         pool.reset();
      }
   }
}
