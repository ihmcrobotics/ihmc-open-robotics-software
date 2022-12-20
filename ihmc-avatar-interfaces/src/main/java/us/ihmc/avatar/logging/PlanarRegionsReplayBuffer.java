package us.ihmc.avatar.logging;

import org.apache.commons.io.IOUtils;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.StringReader;
import java.util.*;

public class PlanarRegionsReplayBuffer<T>
{
   private int buffer_length;
   private HashMap<Integer, Container<T>> indexBuffer;
   private TreeSet<Container<T>> timeBuffer;
   private long firstEverTime = Long.MAX_VALUE;

   private int index = 0;

   private static class Container<T>
   {
      private T list;
      private long time;
      private int index;

      Container(int index, long time, T list)
      {
         this.list = list;
         this.index = index;
         this.time = time;
      }

      public T getList()
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

   public void loadFromLog(File planarRegionListLog, Class<?> type) throws IOException
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

         Object list;

         if(type == PlanarRegionsListWithPose.class)
            list = PlanarRegionFileTools.importPlanarRegionsWithPoseData(temp);
         else
            list = PlanarRegionFileTools.importPlanarRegionData(temp);


         Container container = new Container(index, time, list);

         indexBuffer.put(index, container);
         timeBuffer.add(container);

         firstEverTime = getStartTime();

         index++;
      }

      if (buffer_length < indexBuffer.size())
         buffer_length = indexBuffer.size();

      if (buffer_length <= 0)
      {
         LogTools.warn("Loaded empty log into TBuffer");
      }
   }

   public PlanarRegionsReplayBuffer(File planarRegionListLog, Class<?> type) throws IOException
   {
      loadFromLog(planarRegionListLog, type);
      buffer_length = indexBuffer.size();
   }

   public PlanarRegionsReplayBuffer()
   {
      this(Integer.MAX_VALUE);
   }

   public PlanarRegionsReplayBuffer(int buffer_length)
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

   public void putAndTick(long time, T list)
   {
      Container container = new Container(index, time, list);

      indexBuffer.put(index, container);
      timeBuffer.add(container);

      if (index > buffer_length)
      {
         indexBuffer.remove(index - buffer_length);
         timeBuffer.remove(timeBuffer.first());
      }

      if (time < firstEverTime)
         firstEverTime = time;

      index++;
   }

   public T get(int index)
   {
      Container container = indexBuffer.get(index);
      return container == null ? null : (T) container.getList();
   }

   private Container<T> getNearTimeInternal(long time)
   {
      Container<T> lookup = new Container<T>(-1, time, null);
      Container<T> lower = timeBuffer.lower(lookup);
      Container<T> higher = timeBuffer.higher(lookup);

      Container<T> value;

      if (lower == null)
      {
         if (higher == null)
            return null;
         else
            value = higher;
      }
      else if (higher == null)
         value = lower;
      else if (Math.abs(lower.getTime() - time) > Math.abs(higher.getTime() - time))
         value = higher;
      else
         value = lower;

      return value;
   }

   public T getNearTime(long time)
   {
      Container<T> c = getNearTimeInternal(time);
      return c != null ? (T) c.getList() : null;
   }

   public long getNextTime(long currentTime)
   {
      if (indexBuffer.size() < 1)
         return -1;

      Container<T> container = indexBuffer.get(Objects.requireNonNull(getNearTimeInternal(currentTime + 1)).index + 1); //Increment currentTime by 1 to bias towards higher if tied

      return container == null ? Long.MAX_VALUE : container.getTime();
   }

   public long getPreviousTime(long currentTime)
   {
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

   public long getFirstEverTime()
   {
      return firstEverTime;
   }

   public long getStartTime()
   {
      if (timeBuffer.size() == 0)
         return -1;

      return timeBuffer.first().getTime();
   }

   public long getEndTime()
   {
      if (timeBuffer.size() == 0)
         return -1;

      return timeBuffer.last().getTime();
   }
}
