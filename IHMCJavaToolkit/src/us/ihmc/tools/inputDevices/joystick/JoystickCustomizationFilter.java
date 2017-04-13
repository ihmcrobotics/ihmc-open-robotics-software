package us.ihmc.tools.inputDevices.joystick;

import net.java.games.input.Component.Identifier;
import us.ihmc.tools.inputDevices.joystick.mapping.JoystickMapping;

public class JoystickCustomizationFilter implements JoystickComponentFilter
{
   private final JoystickMapping mapping;
   
   private final boolean invert;
   private final int exponent;
   private final double restToZeroCorrection;
   
   /** The percentage about zero within which the input value should be ignored. */
   private final double deadzone;

   public JoystickCustomizationFilter(JoystickMapping channel, boolean invert)
   {
      this(channel, invert, 0.0);
   }

   public JoystickCustomizationFilter(JoystickMapping channel, boolean invert, double deadzone)
   {
      this(channel, invert, deadzone, 1);
   }

   public JoystickCustomizationFilter(JoystickMapping channel, boolean invert, double deadzone, int exponent)
   {
      this(channel, invert, deadzone, exponent, 0.0);
   }
   
   public JoystickCustomizationFilter(JoystickMapping channel, boolean invert, double deadzone, int exponent, double restToZeroCorrection)
   {
      if (exponent % 2 == 0)
      {
         throw new IllegalArgumentException("Only odd functions are allowed");
      }

      this.mapping = channel;
      this.invert = invert;
      this.deadzone = deadzone;
      this.exponent = exponent;
      this.restToZeroCorrection = restToZeroCorrection;
   }

   @Override
   public double apply(double value)
   {
      value += restToZeroCorrection;
      
      // If the value is within the deadzone, ignore it. Otherwise, offset the value to the deadzone limit and rescale
      // it to full scale.
      if (Math.abs(value) < deadzone)
      {
         value = 0.0;
      }
      else
      {
         value = (value - (Math.signum(value) * deadzone)) / (1.0 - deadzone);
      }

      return Math.pow((invert ? -1 : 1) * value, exponent);
   }

   @Override
   public Identifier getIdentifier()
   {
      return mapping.getIdentifier();
   }
}
