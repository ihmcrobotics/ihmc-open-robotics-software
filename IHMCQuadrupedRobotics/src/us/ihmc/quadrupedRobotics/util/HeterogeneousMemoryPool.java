package us.ihmc.quadrupedRobotics.util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class HeterogeneousMemoryPool
{
   private final Map<Class<?>, HomogeneousMemoryPool<?>> pools = new HashMap<>();
   
   // Cache the values list to prevent allocation in the reset loop.
   private final List<HomogeneousMemoryPool<?>> values = new ArrayList<>();

   @SuppressWarnings("unchecked")
   public <T> T lease(Class<T> type)
   {
      if (pools.containsKey(type))
      {
         return (T) pools.get(type).lease();
      }

      HomogeneousMemoryPool<T> pool = new HomogeneousMemoryPool<>(type);

      pools.put(type, pool);
      values.add(pool);

      return (T) pool.lease();
   }

   public void evict()
   {
      for (HomogeneousMemoryPool<?> pool : values)
      {
         pool.evict();
      }
   }
}
