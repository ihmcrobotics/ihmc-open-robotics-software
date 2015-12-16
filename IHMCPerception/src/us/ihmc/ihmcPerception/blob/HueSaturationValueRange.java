package us.ihmc.ihmcPerception.blob;

import org.opencv.core.Scalar;

public class HueSaturationValueRange
{
   private final double minHue;
   private final double minSaturation;
   private final double minValue;
   private final double maxHue;
   private final double maxSaturation;
   private final double maxValue;
   
   private final Scalar min;
   private final Scalar max;
   
   public HueSaturationValueRange(double minHue, double maxHue, double minSaturation, double maxSaturation, double minValue, double maxValue)
   {
      this.minHue = minHue;
      this.minSaturation = minSaturation;
      this.minValue = minValue;
      this.maxHue = maxHue;
      this.maxSaturation = maxSaturation;
      this.maxValue = maxValue;
      
      min = new Scalar(minHue, minSaturation, minValue);
      max = new Scalar(maxHue, maxSaturation, maxValue);
   }
   
   public Scalar getMin()
   {
      return min;
   }
   
   public Scalar getMax()
   {
      return max;
   }
}
