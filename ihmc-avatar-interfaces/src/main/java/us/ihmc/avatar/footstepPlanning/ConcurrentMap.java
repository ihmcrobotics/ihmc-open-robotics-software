package us.ihmc.avatar.footstepPlanning;

import us.ihmc.concurrent.ConcurrentCopier;

import java.util.HashMap;
import java.util.Map;

class ConcurrentMap<K, V> extends ConcurrentCopier<HashMap<K, V>>
{
   public ConcurrentMap()
   {
      super(HashMap::new);
   }

   public boolean isEmpty()
   {
      Map<K, V> existingMap = getCopyForReading();

      if (existingMap != null)
         return getCopyForReading().isEmpty();
      else
         return true;
   }

   public Iterable<K> iterator()
   {
      Map<K, V> existingMap = getCopyForReading();
      if (existingMap != null)
         return existingMap.keySet();
      else
         return null;
   }

   public void clear()
   {
      Map<K, V> updatedMap = getCopyForWriting();
      updatedMap.clear();
      commit();
   }

   public void put(K key, V value)
   {
      Map<K, V> existingMap = getCopyForReading();
      Map<K, V> updatedMap = getCopyForWriting();
      updatedMap.clear();
      if (existingMap != null)
         updatedMap.putAll(existingMap);
      updatedMap.put(key, value);

      commit();
   }

   public V remove(K key)
   {
      Map<K, V> existingMap = getCopyForReading();
      Map<K, V> updatedMap = getCopyForWriting();
      updatedMap.clear();
      if (existingMap != null)
         updatedMap.putAll(existingMap);
      V objectToReturn = updatedMap.remove(key);

      commit();

      return objectToReturn;
   }
}
