package us.ihmc.robotics.math.exceptions;

public class UndefinedOperationException extends RuntimeException
{
   private static final long serialVersionUID = 3931039918651132184L;

   public UndefinedOperationException(String message)
   {
      super(message);
   }
}