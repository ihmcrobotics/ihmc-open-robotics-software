package us.ihmc.robotbuilder.util;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class CacheTest {

    @Test(timeout = 30000)
    public void testItemsGetStoredInTheCache() {
        Cache<Integer, String> stringCache = new Cache<>(Integer.MAX_VALUE);
        final int numItems = 1000;
        for (int i = 0; i < numItems; i++) {
            stringCache.cacheItem(i, Integer.toString(i));
        }

        for (int i = 0; i < numItems; i++) {
            assertTrue(stringCache.getItem(i).isPresent());
        }
    }

    @Test(timeout = 30000)
    public void testCacheSizeLimit() {
        final int numItems = 200;
        final int cacheSize = numItems / 2;
        Cache<Integer, String> stringCache = new Cache<>(cacheSize);
        for (int i = 0; i < numItems; i++) {
            stringCache.cacheItem(i, Integer.toString(i));
        }

        // First half should be evicted from the cache, second half still in the cache
        for (int i = 0; i < numItems; i++) {
            if (i < cacheSize)
                assertFalse(stringCache.getItem(i).isPresent());
            else
                assertTrue(stringCache.getItem(i).isPresent());
        }
    }

    @Test(timeout = 30000)
    public void testLeastRecentlyUsedGetsRemoved() {
        Cache<Integer, String> stringCache = new Cache<>(3);
        stringCache.cacheItem(1, "1");
        stringCache.cacheItem(2, "2");
        stringCache.cacheItem(3, "3");
        stringCache.getItem(1);
        stringCache.cacheItem(4, "4");

        // 2 is least recently used and should be removed
        assertTrue(stringCache.getItem(1).isPresent());
        assertTrue(stringCache.getItem(3).isPresent());
        assertTrue(stringCache.getItem(4).isPresent());
        assertFalse(stringCache.getItem(2).isPresent());
    }

    @Test(timeout = 30000)
    public void testItemsDoNotGetRemovedEarly() {
        Cache<Integer, String> stringCache = new Cache<>(3);
        stringCache.cacheItem(1, "1");
        stringCache.cacheItem(2, "2");
        stringCache.cacheItem(3, "3");
        stringCache.getItem(1);

        assertTrue(stringCache.getItem(1).isPresent());
        assertTrue(stringCache.getItem(2).isPresent());
        assertTrue(stringCache.getItem(3).isPresent());
    }

    @Test(timeout = 30000)
    public void testCacheHitsAndMissesAreCountedCorrectly() {
        Cache<Integer, String> stringCache = new Cache<>(3);
        stringCache.cacheItem(1, "1");
        stringCache.cacheItem(2, "2");
        stringCache.cacheItem(3, "3");
        stringCache.getItem(1);
        stringCache.getItem(2);
        stringCache.getItem(3);

        assertEquals(stringCache.getHits(), 3);

        stringCache.cacheItem(4, "4");
        stringCache.getItem(1); // should be removed by now

        assertEquals(stringCache.getMisses(), 1);

        stringCache.getItem(42); // does not exist

        assertEquals(stringCache.getMisses(), 2);

        assertEquals(stringCache.getHits(), 3); // number of hits should remain unchanged
    }
}
