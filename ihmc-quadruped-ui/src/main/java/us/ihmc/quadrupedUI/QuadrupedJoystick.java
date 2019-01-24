package us.ihmc.quadrupedUI;

import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;

public class QuadrupedJoystick extends Joystick
{
   public QuadrupedJoystick() throws JoystickNotFoundException
   {
      super(JoystickModel.XBOX_ONE, 0);

      setPollInterval(10);

      setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_TRIGGER, false, 0.1, 1));
      setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_TRIGGER, false, 0.1, 1));
      setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_X, false, 0.1, 1));
      setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_Y, false, 0.1, 1));
      setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_X, false, 0.1, 1));
      setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_Y, false, 0.1, 1));
   }
}
