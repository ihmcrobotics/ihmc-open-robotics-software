package us.ihmc.multicastLogDataProtocol;

public enum LogDataType
{
   DATA(0x41),
   HANDSHAKE(0x12), 
   VIDEO(0x25);
   
   
   private byte header;
   private LogDataType(int header)
   {
      this.header = (byte) header;
   }
   
   public byte getHeader()
   {
      return header;
   }
   
   public static LogDataType[] values = values();
   
   public static LogDataType fromHeader(byte header)
   {
      for(LogDataType type : values)
      {
         if(type.getHeader() == header)
         {
            return type;
         }
      }
      
      return null;
   }
}