package us.ihmc.tools;

import java.util.Arrays;
import java.util.Collections;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BinaryOperator;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collector;

/**
 * Utility class for converting number streams to float arrays as it is not included in Java by default.
 */
public class FloatArrayCollector<From extends Number> implements Collector<From, FloatArrayCollector<From>.Buffer, float[]> {

    class Buffer {
        float[] curr = new float[64];
        int size;

        void add(From d) {
            if (curr.length == size)
                curr = Arrays.copyOf(curr, size * 2);
            curr[size++] = d.floatValue();
        }

        Buffer join(Buffer other) {
            Buffer res = new Buffer();
            res.curr = Arrays.copyOf(curr, size + other.size);
            System.arraycopy(other.curr, 0, res.curr, size, other.size);
            res.size = size + other.size;
            return res;
        }

        float[] toArray() {
            if (size != curr.length)
                curr = Arrays.copyOf(curr, size);
            return curr;
        }
    }

    private FloatArrayCollector() {

    }

    @Override
    public Supplier<Buffer> supplier() {
        return Buffer::new;
    }

    @Override
    public BiConsumer<Buffer, From> accumulator() {
        return Buffer::add;
    }

    @Override
    public BinaryOperator<Buffer> combiner() {
        return Buffer::join;
    }

    @Override
    public Function<Buffer, float[]> finisher() {
        return Buffer::toArray;
    }

    @Override
    public Set<Characteristics> characteristics() {
        return Collections.emptySet();
    }

    public static <T extends Number> Collector<T, ?, float[]> create() {
        return new FloatArrayCollector<>();
    }
}
