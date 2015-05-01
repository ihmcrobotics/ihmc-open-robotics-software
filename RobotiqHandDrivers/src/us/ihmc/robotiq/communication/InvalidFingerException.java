package us.ihmc.robotiq.communication;

public class InvalidFingerException extends RuntimeException
{
   private static final long serialVersionUID = 1887810616332365946L;
   
   public InvalidFingerException(Finger finger)
   {
      super(finger.name() + " is not recognized as a Robotiq finger");
   }
}
