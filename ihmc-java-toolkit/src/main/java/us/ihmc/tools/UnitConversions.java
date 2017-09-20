package us.ihmc.tools;


public class UnitConversions
{
   // Dimensions
   public static final double INCH_TO_METER = 0.0254;
   public static final double SQUAREINCH_TO_SQUAREMETER = INCH_TO_METER * INCH_TO_METER;
   public static final double CUBICINCH_TO_CUBICMETER = INCH_TO_METER * INCH_TO_METER * INCH_TO_METER;

   // Angles
   public static final double DEG_TO_RAD = Math.PI / 180.0;

   // Pressures
   public static final double PSI_TO_PASCALS = 6894.75729;

   // Multi-unit
   public static final double FREQ_TO_RADPERSEC = Math.PI * 2.0;
}
