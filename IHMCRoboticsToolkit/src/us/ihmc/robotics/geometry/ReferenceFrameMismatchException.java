package us.ihmc.robotics.geometry;

/**
 * {@code ReferenceFrameMismatchException} is a {@code RuntimeException} thrown when attempting a
 * forbidden operation because of mismatching {@code ReferenceFrame}s.
 */
public class ReferenceFrameMismatchException extends RuntimeException
{
   private static final long serialVersionUID = -2379052247493923182L;

   /**
    * Creates a new exception with a specified message.
    * 
    * @param message the message detailing the cause of this exception.
    */
   public ReferenceFrameMismatchException(String message)
   {
      super(message);
   }
}
