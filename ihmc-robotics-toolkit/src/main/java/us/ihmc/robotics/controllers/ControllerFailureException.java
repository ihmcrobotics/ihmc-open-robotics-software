package us.ihmc.robotics.controllers;

public class ControllerFailureException extends Exception
{
   private static final long serialVersionUID = 5949754952480608392L;

   public ControllerFailureException(String description)
   {
      super(description);
   }
}
