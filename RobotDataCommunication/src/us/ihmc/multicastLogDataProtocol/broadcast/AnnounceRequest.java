package us.ihmc.multicastLogDataProtocol.broadcast;

import java.nio.ByteBuffer;
import java.util.Arrays;

class AnnounceRequest
{
   enum AnnounceType
   {
      CAN_I_HAZ(0x5), ANNOUNCE(0x12);
      
      private byte header;
      AnnounceType(int header)
      {
         this.header = (byte) header;
      }
      
      public byte getHeader()
      {
         return header;
      }
      
      public static AnnounceRequest.AnnounceType[] values = values();
      public static AnnounceRequest.AnnounceType fromHeader(byte header)
      {
         for(AnnounceRequest.AnnounceType type : values)
         {
            if(type.getHeader() == header)
            {
               return type;
            }
         }
         
         return null;
      }
   }

   private AnnounceRequest.AnnounceType type;
   private long sessionID;
   private byte[] group = new byte[4];
   private byte[] controlIP = new byte[4];
   private short controlPort;
   private String name;

   public AnnounceRequest()
   {
      
   }
   
   public AnnounceRequest(AnnounceRequest original)
   {
      this.type = original.type;
      this.sessionID = original.sessionID;
      System.arraycopy(original.group, 0, group, 0, group.length);
      System.arraycopy(original.controlIP, 0, controlIP, 0, controlIP.length);
      this.controlPort = original.controlPort;
      this.name = new String(original.name);
            
   }
   
   public void readHeader(ByteBuffer buffer)
   {
      if(buffer.remaining() >= 17)
      {
         type = AnnounceType.fromHeader(buffer.get());
         sessionID = buffer.getLong();
         buffer.get(group);
         buffer.get(controlIP);
         controlPort = buffer.getShort();
      }
   }

   public void readName(ByteBuffer buffer)
   {
      if(buffer.hasRemaining())
      {
         int nameLength = buffer.get() & 0xFF;
         if(buffer.remaining() >= nameLength)
         {
            byte[] namebytes = new byte[nameLength];
            buffer.get(namebytes);
            name = new String(namebytes);               
         }
      }
   }

   public ByteBuffer createRequest(ByteBuffer buffer)
   {
      buffer.clear();

      byte[] namebytes = name.getBytes();
      buffer.put((byte) type.getHeader());
      buffer.putLong(sessionID);
      buffer.put(group);
      buffer.put(controlIP);
      buffer.putShort(controlPort);
      buffer.put((byte) namebytes.length);
      buffer.put(namebytes);
      buffer.flip();

      return buffer;
   }

   public AnnounceRequest.AnnounceType getType()
   {
      return type;
   }

   public void setType(AnnounceRequest.AnnounceType type)
   {
      this.type = type;
   }

   public long getSessionID()
   {
      return sessionID;
   }

   public void setSessionID(long sessionID)
   {
      this.sessionID = sessionID;
   }

   public byte[] getGroup()
   {
      return group;
   }

   public void setGroup(byte[] group)
   {
      this.group = group;
   }

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      if (name.length() > 255)
      {
         throw new RuntimeException("Name cannot be longer than 255 characters");
      }
      this.name = name;
   }

   public byte[] getControlIP()
   {
      return controlIP;
   }

   public void setControlIP(byte[] controlIP)
   {
      this.controlIP = controlIP;
   }

   public int getControlPort()
   {
      return controlPort & 0xFFFF;
   }

   public void setControlPort(int controlPort)
   {
      this.controlPort = (short) controlPort;
   }

   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + Arrays.hashCode(controlIP);
      result = prime * result + controlPort;
      result = prime * result + Arrays.hashCode(group);
      result = prime * result + ((name == null) ? 0 : name.hashCode());
      result = prime * result + (int) (sessionID ^ (sessionID >>> 32));
      result = prime * result + ((type == null) ? 0 : type.hashCode());
      return result;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      AnnounceRequest other = (AnnounceRequest) obj;
      if (!Arrays.equals(controlIP, other.controlIP))
         return false;
      if (controlPort != other.controlPort)
         return false;
      if (!Arrays.equals(group, other.group))
         return false;
      if (name == null)
      {
         if (other.name != null)
            return false;
      }
      else if (!name.equals(other.name))
         return false;
      if (sessionID != other.sessionID)
         return false;
      if (type != other.type)
         return false;
      return true;
   }
   
   

}