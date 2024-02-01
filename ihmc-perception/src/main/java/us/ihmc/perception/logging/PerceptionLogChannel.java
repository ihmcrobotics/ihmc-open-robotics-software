package us.ihmc.perception.logging;

import org.bytedeco.javacpp.*;

public class PerceptionLogChannel
{
   public enum ChannelType
   {
      IMAGE_BYTES, TRANSFORM, LONG, DOUBLE, FLOAT, INT, BYTE, BOOLEAN, STRING
   }

   private ChannelType channelType;
   private String name;
   private int count;
   private int index;
   private int blockSize;
   private int frameSize;
   boolean enabled = true;

   private Pointer dataPointer = null;

   public PerceptionLogChannel(String name, int count, int index, int frameSize, int blockSize, Pointer dataPointer)
   {
      this.frameSize = frameSize;
      this.blockSize = blockSize;
      this.name = name;
      this.count = count;
      this.index = index;
      this.dataPointer = dataPointer;
   }

   public boolean isEnabled()
   {
      return enabled;
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

   public String getName()
   {
      return name;
   }

   public int getCount()
   {
      return count;
   }

   public int getIndex()
   {
      return index;
   }

   public void setIndex(int index)
   {
      this.index = index;
   }

   public void incrementIndex()
   {
      index++;
   }

   public void incrementCount()
   {
      count++;
   }

   public void resetIndex()
   {
      index = 0;
   }

   public void resetCount()
   {
      count = 0;
   }

   public void setCount(int count)
   {
      this.count = count;
   }

   public void setDataPointer(Pointer dataPointer)
   {
      this.dataPointer = dataPointer;
   }

   public Pointer getDataPointer()
   {
      return dataPointer;
   }

   public FloatPointer getFloatPointer()
   {
      return (FloatPointer) dataPointer;
   }

   public DoublePointer getDoublePointer()
   {
      return (DoublePointer) dataPointer;
   }

   public LongPointer getLongPointer()
   {
      return (LongPointer) dataPointer;
   }

   public IntPointer getIntPointer()
   {
      return (IntPointer) dataPointer;
   }

   public BytePointer getBytePointer()
   {
      return (BytePointer) dataPointer;
   }

   public int getBlockSize()
   {
      return blockSize;
   }

   public void setBlockSize(int blockSize)
   {
      this.blockSize = blockSize;
   }

   public int getFrameSize()
   {
      return frameSize;
   }

   public void setFrameSize(int frameSize)
   {
      this.frameSize = frameSize;
   }

   public ChannelType getChannelType()
   {
      return channelType;
   }

   public void setChannelType(ChannelType channelType)
   {
      this.channelType = channelType;
   }
}
