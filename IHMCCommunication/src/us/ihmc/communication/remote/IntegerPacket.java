package us.ihmc.communication.remote;

import java.io.Serializable;

public class IntegerPacket implements Serializable
{
   private static final long serialVersionUID = -214863275141788710L;
   private final int contents;

   public IntegerPacket(int contents)
   {
      this.contents = contents;
   }
   
   public int getInt()
   {
      return this.contents;
   }
   
   @Override
   public boolean equals(Object obj)
   {
      if (obj == null) return false;
      
      if (obj.getClass() != this.getClass())
         return false;
      IntegerPacket intPacket = (IntegerPacket) obj;
      return intPacket.contents == this.contents;
   }
   
   public static long getSerialVersionUID()
   {
      return serialVersionUID;
   }
}
