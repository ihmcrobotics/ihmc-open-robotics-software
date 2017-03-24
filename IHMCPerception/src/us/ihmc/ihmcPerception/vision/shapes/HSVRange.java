package us.ihmc.ihmcPerception.vision.shapes;

import org.opencv.core.Scalar;

import us.ihmc.graphicsDescription.color.HSVValue;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HSVRange
{
   /* Common HSV Ranges */
   public static final HSVRange USGAMES_YELLOW_BALL = new HSVRange(new HSVValue(22.84, 81.19, 140.0), new HSVValue(35.0, 136.2, 250.0));
   public static final HSVRange USGAMES_ORANGE_BALL = new HSVRange(new HSVValue(6.5, 124.0, 108.0), new HSVValue(12.05, 168.9, 250.0));
   public static final HSVRange USGAMES_RED_BALL = new HSVRange(new HSVValue(0.2913, 96.09, 69.0), new HSVValue(179.7, 149.6, 190.0));
   public static final HSVRange USGAMES_GREEN_BALL = new HSVRange(new HSVValue(81.29, 103.0, 52.0), new HSVValue(91.15, 153.0, 130.0));
   public static final HSVRange USGAMES_BLUE_BALL = new HSVRange(new HSVValue(79.5, 109.3, 28.0), new HSVValue(105.0, 175.9, 170.0));
   public static final HSVRange SIMULATED_BALL = new HSVRange(new HSVValue(20.0, 50.0, 0.0), new HSVValue(150.0, 255.0, 255.0));
   
   private final Scalar lowerBoundScalar;
   private final Scalar upperBoundScalar;
   private final HSVValue lowerBoundHSV;
   private final HSVValue upperBoundHSV;

   public HSVRange(HSVValue lowerBound, HSVValue upperBound)
   {
      this.lowerBoundHSV = lowerBound;
      this.upperBoundHSV = upperBound;
      this.lowerBoundScalar = new Scalar(lowerBound.getHue(), lowerBound.getSaturation(), lowerBound.getBrightnessValue());
      this.upperBoundScalar = new Scalar(upperBound.getHue(), upperBound.getSaturation(), upperBound.getBrightnessValue());
   }

   public Scalar getLowerBoundOpenCVScalar()
   {
      return lowerBoundScalar;
   }

   public Scalar getUpperBoundOpenCVScalar()
   {
      return upperBoundScalar;
   }

   public HSVValue getLowerBound()
   {
      return lowerBoundHSV;
   }

   public HSVValue getUpperBound()
   {
      return upperBoundHSV;
   }

   @Override
   public boolean equals(Object o)
   {
      if (this == o)
         return true;
      if (o == null || getClass() != o.getClass())
         return false;

      HSVRange that = (HSVRange) o;

      if (!lowerBoundScalar.equals(that.lowerBoundScalar))
         return false;
      if (!upperBoundScalar.equals(that.upperBoundScalar))
         return false;
      if (!lowerBoundHSV.equals(that.lowerBoundHSV))
         return false;
      return upperBoundHSV.equals(that.upperBoundHSV);

   }
   
   @Override
   public String toString()
   {
      return HSVRange.class.getSimpleName() + ": Lower: " + lowerBoundHSV + " Upper: " + upperBoundHSV;
   }

   @Override
   public int hashCode()
   {
      int result = lowerBoundScalar.hashCode();
      result = 31 * result + upperBoundScalar.hashCode();
      result = 31 * result + lowerBoundHSV.hashCode();
      result = 31 * result + upperBoundHSV.hashCode();
      return result;
   }
}
