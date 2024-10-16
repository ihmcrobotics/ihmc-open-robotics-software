package us.ihmc.commons;

/**
 * Staging grounds for the ihmc-commons Conversions class.
 */
public class UnitConversions
{
   // Dimensions
   public static final double INCH_TO_METER = 0.0254;
   public static final double SQUAREINCH_TO_SQUAREMETER = INCH_TO_METER * INCH_TO_METER;
   public static final double CUBICINCH_TO_CUBICMETER = INCH_TO_METER * INCH_TO_METER * INCH_TO_METER;
   public static final String DEGREE_SYMBOL = "\u00B0";

   // Angles
   public static final double DEG_TO_RAD = Math.PI / 180.0;

   // Pressures
   public static final double PSI_TO_PASCALS = 6894.75729;

   // Multi-unit
   /**
    * @see {@link us.ihmc.commons.Conversions#radiansPerSecondToHertz(double)}
    */
   public static final double FREQ_TO_RADPERSEC = Math.PI * 2.0;

   public static double secondsToHertz(double seconds)
   {
      return 1.0 / seconds;
   }

   public static double hertzToSeconds(double hertz)
   {
      return 1.0 / hertz;
   }

   public static double inchesToMeters(double inches)
   {
      return inches * INCH_TO_METER;
   }

   public static double metersToInches(double meters)
   {
      return meters / INCH_TO_METER;
   }
}
