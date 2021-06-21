package us.ihmc.avatar.logging;

import org.apache.commons.io.IOUtils;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.StringReader;
import java.util.*;

public class PlanarRegionsListBuffer
{
   private int buffer_length;
   private HashMap<Integer, Container> indexBuffer;
   private TreeSet<Container> timeBuffer;

   private int index = 0;
   private long start_time;
   private long end_time;

   private static class Container {
      private PlanarRegionsList list;
      private long time;
      private int index;

      Container(int index, long time, PlanarRegionsList list) {
         this.list = list;
         this.index = index;
         this.time = time;
      }

      public PlanarRegionsList getList()
      {
         return list;
      }

      public long getTime()
      {
         return time;
      }

      public int getIndex()
      {
         return index;
      }
   }

   private static final Comparator<Container> customCompare = (o1, o2) ->
   {
      long compare = o1.getTime() - o2.getTime();
      if (compare > 0)
         return 1;
      else if (compare == 0)
         return 0;
      else
         return -1;
   };

   public void loadFromLog(File planarRegionListLog) throws IOException
   {
      indexBuffer = new HashMap<>();
      timeBuffer = new TreeSet<>(customCompare);

      Scanner in = new Scanner(planarRegionListLog);
      in.useDelimiter("##\n");

      in.next(); //Skip metadata TODO process metadata
      int index = 0;
      while (in.hasNext())
      {
         in.nextLine(); //Skip past delimiter
         long time = Long.parseLong(in.nextLine());

         final File temp = File.createTempFile("prll", ".tmp");
         temp.deleteOnExit();
         try (FileOutputStream out = new FileOutputStream(temp))
         {
            IOUtils.copy(new StringReader(in.next()), out);
         }
         PlanarRegionsList list = PlanarRegionFileTools.importPlanarRegionData(temp);

         Container container = new Container(index, time, list);

         indexBuffer.put(index, container);
         timeBuffer.add(container);

         index++;
      }

      if (buffer_length < indexBuffer.size())
         buffer_length = indexBuffer.size();

      start_time = timeBuffer.first().getTime();
      end_time = timeBuffer.last().getTime();
   }

   public PlanarRegionsListBuffer(File planarRegionListLog) throws IOException
   {
      loadFromLog(planarRegionListLog);
      buffer_length = indexBuffer.size();
   }

   public PlanarRegionsListBuffer()
   {
      this(Integer.MAX_VALUE);
   }

   public PlanarRegionsListBuffer(int buffer_length)
   {
      this.buffer_length = buffer_length;
      this.indexBuffer = new HashMap<>();
      this.timeBuffer = new TreeSet<>(customCompare);
   }

   public void expandBuffer(long additionalSize)
   {
      if (additionalSize <= 0)
         return;

      buffer_length += additionalSize;
   }

   public void putAndTick(long time, PlanarRegionsList list)
   {
      Container container = new Container(index, time, list);

      indexBuffer.put(index, container);
      timeBuffer.add(container);

      if (index > buffer_length)
      {
         indexBuffer.remove(index - buffer_length);
         timeBuffer.remove(timeBuffer.first());
      }

      if (time < start_time)
         start_time = time;
      else if (time > end_time)
         end_time = time;

      index++;
   }

   public PlanarRegionsList get(int index)
   {
      Container container = indexBuffer.get(index);
      return container == null ? null : container.getList();
   }

   private Container getNearTimeInternal(long time)
   {
      Container lookup = new Container(-1, time, null);
      Container lower = timeBuffer.lower(lookup);
      Container higher = timeBuffer.higher(lookup);

      Container value;

      if (lower == null) {
         if (higher == null)
            return null;
         else
            value = higher;
      } else if (higher == null)
         value = lower;
      else if (Math.abs(lower.getTime() - time) > Math.abs(higher.getTime() - time))
         value = higher;
      else
         value = lower;

      return value;
   }

   public PlanarRegionsList getNearTime(long time) {
      Container c = getNearTimeInternal(time);
      return c != null ? c.getList() : null;
   }

   public long getNextTime(long currentTime) {
      if (indexBuffer.size() < 1)
         return -1;

      Container container = indexBuffer.get(getNearTimeInternal(currentTime + 1).index + 1); //Increment currentTime by 1 to bias towards higher if tied

      return container == null ? Long.MAX_VALUE : container.getTime();
   }

   public long getPreviousTime(long currentTime) {
      if (indexBuffer.size() < 1)
         return -1;

      Container container = indexBuffer.get(getNearTimeInternal(currentTime - 1).index - 1); //Decrement currentTime by 1 to bias towards lower if tied

      return container == null ? 0 : container.getTime();
   }

   public long getCurrentIndex()
   {
      return index;
   }

   public int getBufferLength()
   {
      return buffer_length;
   }

   public long getStartTime()
   {
      return start_time;
   }

   public long getEndTime()
   {
      return end_time;
   }
}
