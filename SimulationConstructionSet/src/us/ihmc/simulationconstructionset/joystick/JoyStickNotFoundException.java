package us.ihmc.simulationconstructionset.joystick;

public class JoyStickNotFoundException extends RuntimeException
{
   public JoyStickNotFoundException()
   {
      super ("joystick not found");
   }
}