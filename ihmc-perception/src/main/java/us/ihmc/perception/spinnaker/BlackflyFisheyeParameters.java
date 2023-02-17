package us.ihmc.perception.spinnaker;

import us.ihmc.tools.UnitConversions;

public class BlackflyFisheyeParameters
{
   // Fujinon FE185C086HA_1 lens
   // https://www.fujifilm.com/us/en/business/optical-devices/machine-vision-lens/fe185-series
   private static final double FE185C086HA_1_FOCAL_LENGTH = 0.0027;
   // https://www.flir.com/products/blackfly-usb3/?model=BFLY-U3-23S6C-C&vertical=machine+vision&segment=iis
   private static final double BFLY_U3_23S6C_CMOS_SENSOR_FORMAT = UnitConversions.inchesToMeters(1.0 / 1.2);
   private static final double BFLY_U3_23S6C_CMOS_SENSOR_WIDTH = 0.01067;
   private static final double BFLY_U3_23S6C_CMOS_SENSOR_HEIGHT = 0.00800;
   public static final double BFLY_U3_23S6C_WIDTH_PIXELS = 1920.0;
   public static final double FE185C086HA_1_FOCAL_LENGTH_IN_BFLY_U3_23S6C_PIXELS
         = FE185C086HA_1_FOCAL_LENGTH * BFLY_U3_23S6C_WIDTH_PIXELS / BFLY_U3_23S6C_CMOS_SENSOR_WIDTH;
   public static final double BFLY_U3_23S6C_HEIGHT_PIXELS = 1200.0;

   public static final double FOCAL_LENGTH_X_FOR_COLORING = 472.44896;   // These were tuned with sliders on the benchtop
   public static final double FOCAL_LENGTH_Y_FOR_COLORING = 475.51022;   // by Bhavyansh and Duncan and copied here
   public static final double PRINCIPAL_POINT_X_FOR_COLORING = 970.06801;// by hand.
   public static final double PRINCIPAL_POINT_Y_FOR_COLORING = 608.84360;
}
