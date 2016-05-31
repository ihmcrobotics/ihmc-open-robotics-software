package us.ihmc.ihmcPerception.vision.shapes;

import org.opencv.core.Scalar;
import us.ihmc.ihmcPerception.vision.HSVValue;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HSVRange
{
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
