package us.ihmc.simulationconstructionset.joystick;

import java.util.ArrayList;

import us.ihmc.simulationconstructionset.util.inputdevices.EnumDependentInputMapping;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;

public interface EnumDependentJoystickMapping<T extends Enum<T>> extends EnumDependentInputMapping<T>
{
   public ArrayList<JoystickEventListener> getEventListeners();
}
