package us.ihmc.quadrupedRobotics.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A memory pool capable of allocating and leasing objects of any type. Objects
 * are individually borrowed with the {@link #lease(Class)} method, and the
 * entire pool is returned with the {@link #evict()} method.
 *
 * This is useful in a tight control loop where local variables cannot be
 * allocated and it is known that references will not be persisted across loop
 * iterations, so {@link #evict()} can safely be called.
 */
public class HeterogeneousMemoryPool
{
   private final Map<Class<?>, HomogeneousMemoryPool<?>> pools = new HashMap<>();

   // Cache the values list to prevent allocation in the reset loop.
   private final List<HomogeneousMemoryPool<?>> values = new ArrayList<>();

   /**
    * Borrow an object from the pool. An allocation will occur only if an object
    * of this type does not already exist in the pool.
    *
    * Note that leased objects are only valid until {@link #evict()} is called,
    * so references should not be held onto past that time.
    *
    * @param type
    *           the desired object type.
    * @return the borrowed object.
    */
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

      return pool.lease();
   }

   /**
    * Invalidate all leased pool objects.
    */
   public void evict()
   {
      for (HomogeneousMemoryPool<?> pool : values)
      {
         pool.evict();
      }
   }
}
