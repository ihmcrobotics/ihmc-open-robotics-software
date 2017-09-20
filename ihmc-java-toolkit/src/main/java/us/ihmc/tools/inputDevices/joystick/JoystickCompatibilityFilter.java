package us.ihmc.tools.inputDevices.joystick;

import net.java.games.input.Component.Identifier;
import us.ihmc.tools.inputDevices.joystick.mapping.JoystickMapping;

public class JoystickCompatibilityFilter implements JoystickComponentFilter
{
   private final JoystickMapping mapping;
   
   private final boolean invert;
   private final double restToZeroCorrection;
   private final double scaleCorrection;
   
   public JoystickCompatibilityFilter(JoystickMapping channel, boolean invert, double restToZeroCorrection, double scaleCorrection)
   {
      this.mapping = channel;
      this.invert = invert;
      this.restToZeroCorrection = restToZeroCorrection;
      this.scaleCorrection = scaleCorrection;
   }

   @Override
   public double apply(double value)
   {
      double corrected = value + restToZeroCorrection;
      double scaled = corrected * scaleCorrection;
      return invert ? -scaled : scaled;
   }

   @Override
   public Identifier getIdentifier()
   {
      return mapping.getIdentifier();
   }
}
