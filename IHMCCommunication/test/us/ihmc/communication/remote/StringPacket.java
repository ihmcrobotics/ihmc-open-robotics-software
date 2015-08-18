package us.ihmc.communication.remote;

import java.io.Serializable;

public class StringPacket implements Serializable
{
   private static final long serialVersionUID = 413928572816648643L;
   private String contents = "this should be serializable";

   public StringPacket(String contents)
   {
      this.contents = contents;
   }

   public boolean equals(StringPacket obj)
   {
      return obj.contents.equals(this.contents);
   }
   
   public String getString()
   {
      return contents;
   }
   
   @Override
   public boolean equals(Object object)
   {
      if (object == null) return false;
      if (!(object instanceof StringPacket)) return false;
      if (object == this) return true;

      StringPacket intPacket = (StringPacket) object;
      return intPacket.contents.equals(this.contents);
   }

   public static long getSerialVersionUID()
   {
      return serialVersionUID;
   }
}
