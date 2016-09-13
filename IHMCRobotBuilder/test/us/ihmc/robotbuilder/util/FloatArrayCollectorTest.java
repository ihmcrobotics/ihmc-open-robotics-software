package us.ihmc.robotbuilder.util;

import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.stream.IntStream;

import static org.junit.Assert.*;

public class FloatArrayCollectorTest {
    @Test
    public void testCollectZeroItemsToAZeroLengthArray() {
        float[] res = Collections.<Float>emptyList().stream().collect(new FloatArrayCollector<>());
        assertEquals(0, res.length);
    }

    @Test
    public void testCollectOneItem() {
        float[] res = Collections.singletonList(1.0).stream().collect(new FloatArrayCollector<>());
        assertEquals(1, res.length);
        assertEquals(1.0f, res[0], 1e-5);
    }

    @Test
    public void testCollectManyItems() {
        float[] res = Arrays.stream(new double[8192]).mapToObj(Double::new).collect(new FloatArrayCollector<>());
        assertEquals(res.length, 8192);
        float sum = 0;
        for (float re : res)
            sum += re;
        assertEquals(0, sum, 1e-5);
    }

    @Test
    public void testCollectParallel() {
        final int n = 8192;
        float[] res = IntStream.iterate(1, i -> i + 1).limit(n).mapToObj(Double::new).parallel().collect(new FloatArrayCollector<>());
        assertEquals(res.length, n);
        double sum = 0;
        for (float re : res)
            sum += re;
        assertEquals(n * (n + 1) / 2, sum, 1e-5);
    }
}
