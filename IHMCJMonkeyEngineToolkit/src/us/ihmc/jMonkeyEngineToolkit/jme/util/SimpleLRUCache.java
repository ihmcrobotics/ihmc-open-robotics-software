package us.ihmc.jMonkeyEngineToolkit.jme.util;

import java.util.LinkedHashMap;
import java.util.Map;

public class SimpleLRUCache<K, V> extends LinkedHashMap<K, V>
{
   private static final long serialVersionUID = -8670808345162245362L;
   private final int maxEntries;

   public SimpleLRUCache(int maxEntries)
   {
      super(maxEntries + 1, 1.0f, true);
      this.maxEntries = maxEntries;
   }

   @Override
   protected boolean removeEldestEntry(final Map.Entry<K, V> eldest)
   {
      return super.size() > maxEntries;
   }
}