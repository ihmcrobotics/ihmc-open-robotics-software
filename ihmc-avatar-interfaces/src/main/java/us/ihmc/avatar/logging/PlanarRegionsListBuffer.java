package us.ihmc.avatar.logging;

import org.apache.commons.io.IOUtils;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.StringReader;
import java.util.HashMap;
import java.util.Scanner;

public class PlanarRegionsListBuffer
{
   private long buffer_length = 0;
   private HashMap<Long, PlanarRegionsList> buffer;
   private long index = 0;

   public void loadFromLog(File planarRegionListLog) throws IOException
   {
      buffer = new HashMap<>();
      Scanner in = new Scanner(planarRegionListLog);
      in.useDelimiter("##\n");

      in.next(); //Skip metadata
      while (in.hasNext()) {
         in.nextLine(); //Skip past delimiter
         long id = Long.parseLong(in.nextLine());

         final File temp = File.createTempFile("prll", ".tmp");
         temp.deleteOnExit();
         try (FileOutputStream out = new FileOutputStream(temp)) {
            IOUtils.copy(new StringReader(in.next()), out);
         }
         PlanarRegionsList list = PlanarRegionFileTools.importPlanarRegionData(temp);

         buffer.put(id, list);
      }

      if (buffer_length < buffer.size())
         buffer_length = buffer.size();
   }

   public PlanarRegionsListBuffer(File planarRegionListLog) throws IOException
   {
      loadFromLog(planarRegionListLog);
   }

   public PlanarRegionsListBuffer()
   {
      this(Long.MAX_VALUE);
   }

   public PlanarRegionsListBuffer(long buffer_length)
   {
      this.buffer_length = buffer_length;
      this.buffer = new HashMap<>();
   }

   public void expandBuffer(long additionalSize) {
      if (additionalSize <= 0)
         return;

      buffer_length += additionalSize;
   }

   public void putAndTick(PlanarRegionsList list)
   {
      buffer.put(index, list);

      if (index > buffer_length)
         buffer.remove(index - buffer_length);

      index++;
   }

   public PlanarRegionsList get(long index)
   {
      return buffer.get(index);
   }

   public long getCurrentIndex()
   {
      return index;
   }

   public long getBufferLength()
   {
      return buffer_length;
   }
}
