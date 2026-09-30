package us.ihmc.gr00t;

import java.nio.DoubleBuffer;
import java.util.List;

/** Decodes validated model-specific rows from a GR00T action tensor. */
@FunctionalInterface
public interface Gr00tActionDecoder<T>
{
   List<T> decode(DoubleBuffer actionChunk, int realActionCount);
}
