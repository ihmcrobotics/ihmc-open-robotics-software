package us.ihmc.gr00t;

import java.nio.ByteBuffer;

/** Packs one coherent robot state into the GR00T wire buffer. */
@FunctionalInterface
public interface Gr00tStatePacker
{
   boolean pack(ByteBuffer stateBuffer);
}
