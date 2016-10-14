package us.ihmc.robotbuilder.util;

import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.StreamSupport;

/**
 *
 */
public class TreeAdapter<Source> implements Tree<TreeAdapter<Source>> {
    private final Source value;
    private final Function<Source, ? extends Iterable<? extends Source>> childGetter;

    public TreeAdapter(Source value, Function<Source, ? extends Iterable<? extends Source>> childGetter) {
        this.value = value;
        this.childGetter = childGetter;
    }

    @Override
    public Iterable<TreeAdapter<Source>> getChildren() {
        return StreamSupport.stream(childGetter.apply(value).spliterator(), false)
                .map(child -> new TreeAdapter<>(child, childGetter))
                .collect(Collectors.toList());
    }

    public Source getValue() {
        return value;
    }
}