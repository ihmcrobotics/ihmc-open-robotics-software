package us.ihmc.communication.remote;

public class UnsupportedFormatException extends Exception
{
   private static final long serialVersionUID = 8845641123167087257L;

   public UnsupportedFormatException()
   {
   }

   public UnsupportedFormatException(String msg)
   {
      super(msg);
   }
}
