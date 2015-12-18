package us.ihmc.quadrupedRobotics;

/**
 * An exception that is thrown when a request is made for a joint that does not
 * exist on the given robot.
 */
public class NonexistentJointException extends RuntimeException
{
   private static final long serialVersionUID = -1904127645147139075L;

   public NonexistentJointException(String jointName)
   {
      super("Joint does not exist: " + jointName);
   }
}
