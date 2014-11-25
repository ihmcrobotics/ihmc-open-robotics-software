package us.ihmc.multicastLogDataProtocol;

import java.nio.ByteBuffer;

public class SegmentHeader
{

   public static final int HEADER_SIZE = 29;

   private LogDataType type;
   private long sessionID;
   private long packageID;

   private short segmentCount;
   private short segmentID;

   /*
    * Timestamp is in every segment header so that additional data can be
    * synchronized as soon as one packet comes in
    */
   private long timestamp;

   public void write(ByteBuffer destination)
   {
      destination.put((byte) type.getHeader());

      destination.putLong(sessionID);
      destination.putLong(packageID);
      destination.putShort(segmentCount);
      destination.putShort(segmentID);
      destination.putLong(timestamp);
   }

   public LogDataType getType()
   {
      return type;
   }

   public void setType(LogDataType type)
   {
      this.type = type;
   }

   public short getSegmentCount()
   {
      return segmentCount;
   }

   public void setSegmentCount(int segmentCount)
   {
      this.segmentCount = (short) segmentCount;
   }

   public int getSegmentID()
   {
      return segmentID;
   }

   public void setSegmentID(int segmentID)
   {
      this.segmentID = (short) segmentID;
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }

   public long getPackageID()
   {
      return packageID;
   }

   public void setPackageID(long uid)
   {
      this.packageID = uid;
   }

   public long getSessionID()
   {
      return sessionID;
   }

   public void setSessionID(long sessionID)
   {
      this.sessionID = sessionID;
   }

   public boolean read(ByteBuffer receiveBuffer)
   {
      if(receiveBuffer.remaining() < HEADER_SIZE)
      {
         return false;
      }
      
      setType(LogDataType.fromHeader(receiveBuffer.get()));
      if(getType() == null)
      {
         return false;
      }
      sessionID = receiveBuffer.getLong();
      packageID = receiveBuffer.getLong();
      segmentCount = receiveBuffer.getShort();
      segmentID = receiveBuffer.getShort();
      timestamp = receiveBuffer.getLong();
      
      return true;
   }

}
