package us.ihmc.ihmcPerception.vision;

import java.util.Arrays;

import us.ihmc.robotics.MathTools;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HSVValue
{
   private final double[] values = new double[3];

   public HSVValue(double hue, double saturation, double brightnessValue)
   {
      values[0] = hue;
      values[1] = saturation;
      values[2] = brightnessValue;
   }

   public void setHue(double hueValue)
   {
      values[0] = hueValue;
   }

   public void setSaturation(double saturationValue)
   {
      values[1] = saturationValue;
   }

   public void setBrightnessValue(double brightnessValue)
   {
      values[2] = brightnessValue;
   }

   public double getHue()
   {
      return values[0];
   }

   public double getSaturation()
   {
      return values[1];
   }

   public double getBrightnessValue()
   {
      return values[2];
   }

   @Override
   public boolean equals(Object o)
   {
      if (this == o)
         return true;
      if (o == null || getClass() != o.getClass())
         return false;

      HSVValue hsvValue = (HSVValue) o;

      return Arrays.equals(values, hsvValue.values);
   }
   
   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": H: " + MathTools.roundToSignificantFigures(values[0], 4) + " S: "
            + MathTools.roundToSignificantFigures(values[1], 4) + " V: " + MathTools.roundToSignificantFigures(values[2], 4);
   }

   @Override
   public int hashCode()
   {
      return Arrays.hashCode(values);
   }
}
