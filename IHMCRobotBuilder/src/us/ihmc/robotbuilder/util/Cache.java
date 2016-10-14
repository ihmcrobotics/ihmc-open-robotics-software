package us.ihmc.robotbuilder.util;

import java.util.*;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Supplier;

/**
 * A simple, thread-safe LRU cache class for caching various resources.
 */
public class Cache<Key, Value> {

    private static AtomicLong actionCounter = new AtomicLong();
    private final Map<Key, CachedItem> cache;
    private final int limit;

    private long hits, misses;

    /**
     * Creates a new cache with the given size limit. The Map supplier is used to create the internal
     * map, the user can choose the most efficient implementation for their use case.
     * @param limit maximum number of items in the cache
     * @param mapSupplier constructor for the internal map
     */
    public Cache(int limit, Supplier<Map<Key, CachedItem>> mapSupplier) {
        this.limit = limit;
         this.cache = mapSupplier.get();
    }

    /**
     * Creates a new cache with the given size limit.
     * @param limit maximum number of items in the cache
     */
    public Cache(int limit) {
        this(limit, HashMap::new);
    }

    /**
     * Insert an item into the cache
     * @param key cache key
     * @param value cached value
     */
    public synchronized void cacheItem(Key key, Value value) {
        cache.put(key, new CachedItem(value, actionCounter.getAndIncrement()));
        pruneCache();
    }

    /**
     * Retrieve an item from the cache
     * @param key key to retrieve
     * @return cached value or {@link Optional#empty()} if the value is not present in the cache
     */
    public synchronized Optional<Value> getItem(Key key) {
        CachedItem item = cache.get(key);
        if (item != null) {
            item.lastUsed = actionCounter.getAndIncrement();
            ++hits;
            return Optional.of(item.value);
        }
        ++misses;
        return Optional.empty();
    }

    /**
     * Get the number of cache hits since the cache was created
     * @return cache hits
     */
    public long getHits() {
        return hits;
    }

    /**
     * Get the number of cache misses since the cache was created
     * @return cache misses
     */
    public long getMisses() {
        return misses;
    }

    private synchronized void pruneCache() {
        int removeCount = Math.max(0, cache.size() - limit);
        if (removeCount == 0)
            return;

        List<Map.Entry<Key, CachedItem>> entries = new ArrayList<>(cache.entrySet());
        Collections.sort(entries, (e1, e2) -> Long.compare(e1.getValue().lastUsed, e2.getValue().lastUsed));

        for (int i = 0; i < removeCount; i++) {
            cache.remove(entries.get(i).getKey());
        }
    }

    public class CachedItem {
        private Value value;
        private long lastUsed;

        CachedItem(Value value, long lastUsed) {
            this.value = value;
            this.lastUsed = lastUsed;
        }
    }
}
