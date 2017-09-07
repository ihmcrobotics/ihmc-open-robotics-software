package us.ihmc.robotDataLogger;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

public class LogIndex
{
   public long[] timestamps;
   public long[] dataOffsets;
   public int[] compressedSizes;
   private final int numberOfEntries;

   public LogIndex(File indexData, long channelSize) throws IOException
   {
      FileInputStream indexStream = new FileInputStream(indexData);
      FileChannel indexChannel = indexStream.getChannel();
      timestamps = new long[(int) (indexChannel.size() / 16)];
      dataOffsets = new long[(int) (indexChannel.size() / 16)];

      int index = 0;
      ByteBuffer indexDataBuffer = ByteBuffer.allocateDirect(16);
      while (indexChannel.read(indexDataBuffer) == 16)
      {
         indexDataBuffer.clear();
         timestamps[index] = indexDataBuffer.getLong(0);
         dataOffsets[index] = indexDataBuffer.getLong(8);
         index++;
      }
      indexChannel.close();
      indexStream.close();

      compressedSizes = new int[dataOffsets.length];
      for (int i = 0; i < dataOffsets.length - 1; i++)
      {
         compressedSizes[i] = (int) (dataOffsets[i + 1] - dataOffsets[i]);
      }
      compressedSizes[dataOffsets.length - 1] = (int) (channelSize - dataOffsets[dataOffsets.length - 1]);
      numberOfEntries = dataOffsets.length;

   }

   public int seek(long inStamp) throws IOException
   {
      int head = 0;
      int tail = numberOfEntries;
      int position = -1;

      while (head < tail)
      {
         position = head + (tail - head) / 2;
         long timestamp = timestamps[position];

         if (timestamp < inStamp)
         {
            head = position + 1;
         }
         else
         {
            tail = position;
         }
      }
      return position;
   }

   public int getNumberOfEntries()
   {
      return dataOffsets.length;
   }

   public long getInitialTimestamp()
   {
      return timestamps[0];
   }
}
