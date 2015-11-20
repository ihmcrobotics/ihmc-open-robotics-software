package us.ihmc.simulationconstructionset.joystick;

import java.util.ArrayList;

import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;

public interface EnumDependentJoystickMapping
{
   public ArrayList<JoystickEventListener> getEventListeners();
   
   public Enum<?> getEnum();
}
