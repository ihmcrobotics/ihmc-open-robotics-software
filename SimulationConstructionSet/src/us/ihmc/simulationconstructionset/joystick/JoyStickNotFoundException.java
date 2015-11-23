package us.ihmc.simulationconstructionset.joystick;

@SuppressWarnings("serial")
public class JoyStickNotFoundException extends RuntimeException
{
   public JoyStickNotFoundException()
   {
      super ("Joystick not found.");
   }
}