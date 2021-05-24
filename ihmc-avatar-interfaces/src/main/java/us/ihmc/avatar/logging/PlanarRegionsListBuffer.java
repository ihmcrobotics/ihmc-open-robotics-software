package us.ihmc.avatar.logging;

import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.HashMap;

public class PlanarRegionsListBuffer
{
   private final long buffer_length;
   private final HashMap<Long, PlanarRegionsList> buffer;
   private long index = 0;

   public PlanarRegionsListBuffer() {
      this(Long.MAX_VALUE);
   }

   public PlanarRegionsListBuffer(long buffer_length) {
      this.buffer_length = buffer_length;
      this.buffer = new HashMap<>();
   }

   public void putAndTick(PlanarRegionsList list) {
      buffer.put(index, list);

      if (index > buffer_length)
         buffer.remove(index - buffer_length);

      index++;
   }

   public PlanarRegionsList get(long index) {
      return buffer.get(index);
   }

   public long getCurrentIndex() {
      return index;
   }

   public long getBufferLength() {
      return buffer_length;
   }
}
