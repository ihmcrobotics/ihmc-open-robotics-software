package us.ihmc.robotDataLogger;

import java.nio.ByteBuffer;

public class LogDataHeader
{
   @Override
   public String toString()
   {
      return "LogDataHeader [uid=" + uid + ", timestamp=" + timestamp + ", dataSize=" + dataSize + ", crc32=" + crc32 + "]";
   }
   
   public static final byte KEEP_ALIVE_PACKET = 0x11;
   public static final byte DATA_PACKET = 0x22;
   public static final byte VIDEO_PACKET = 0x33;

   public static final short HEADER = 0x7A7A;
   private long uid;
   private long timestamp;
   private byte type;
   private int dataSize;
   private int crc32;

   public static int length()
   {
      return 2 /* HEADER */+ 8 /* uid */ + 8 /* timestamp */ + 1 /* type */ + 4 /* length */+ 4 /* crc32 */;
   }

   public boolean readBuffer(ByteBuffer buffer)
   {
      if (buffer.getShort() != HEADER)
      {
         return false;
      }
      uid = buffer.getLong();
      timestamp = buffer.getLong();
      type = buffer.get();
      dataSize = buffer.getInt();
      crc32 = buffer.getInt();
      if(dataSize < 0)
      {
         return false;
      }
      return true;
   }

   public void writeBuffer(int pos, ByteBuffer buffer)
   {
      buffer.putShort(pos, HEADER);
      buffer.putLong(pos + 2, uid);
      buffer.putLong(pos + 10, timestamp);
      buffer.put(pos + 18, type);
      buffer.putInt(pos + 19, dataSize);
      buffer.putInt(pos + 23, crc32);
   }

   public long getUid()
   {
      return uid;
   }

   public void setUid(long uid)
   {
      this.uid = uid;
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }
   
   public byte getType()
   {
      return type;
   }
   
   public void setType(byte type)
   {
      this.type = type;
   }

   public int getDataSize()
   {
      return dataSize;
   }

   public void setDataSize(int dataSize)
   {
      this.dataSize = dataSize;
   }

   public int getCrc32()
   {
      return crc32;
   }

   public void setCrc32(int crc32)
   {
      this.crc32 = crc32;
   }

}
