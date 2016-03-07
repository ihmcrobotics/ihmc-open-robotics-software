package us.ihmc.communication.streamingData;

import java.io.Serializable;

public class SimpleStreamedData implements Serializable
{
   private static final long serialVersionUID = 8313773654191190644L;
   private final int value;
   
   public SimpleStreamedData(int value)
   {
      this.value = value;
   }

   public int getValue()
   {
      return value;
   }
   
   public static long getSerialVersionUID()
   {
      return serialVersionUID;
   }
}
