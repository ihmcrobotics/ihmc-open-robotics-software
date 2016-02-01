package us.ihmc.robotics.geometry;

public class ReferenceFrameMismatchException extends RuntimeException
{
   private static final long serialVersionUID = -2379052247493923182L;

   public ReferenceFrameMismatchException(String message)
   {
      super(message);
   }
}
