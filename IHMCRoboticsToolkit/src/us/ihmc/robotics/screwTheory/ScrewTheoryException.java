package us.ihmc.robotics.screwTheory;

/**
 * {@code ScrewTheoryException} is used for handling exception thrown from the the screw theory
 * library.
 */
public class ScrewTheoryException extends RuntimeException
{
   private static final long serialVersionUID = -4504468232252668130L;

   public ScrewTheoryException(String message)
   {
      super(message);
   }
}
