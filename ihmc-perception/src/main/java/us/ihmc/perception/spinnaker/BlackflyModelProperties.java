package us.ihmc.perception.spinnaker;

import us.ihmc.tools.UnitConversions;

/**
 * The CMOS sensor sizes are taken from the table at
 * https://en.wikipedia.org/wiki/Image_sensor_format#Table_of_sensor_formats_and_sizes
 */
public enum BlackflyModelProperties
{
   // https://www.flir.com/products/blackfly-usb3/?model=BFLY-U3-23S6C-C&vertical=machine+vision&segment=iis
   BFLY_U3_23S6C(1920.0, 1200.0, UnitConversions.inchesToMeters(1.0 / 1.2), 0.01067, 0.00800),
   // https://www.flir.com/products/blackfly-s-usb3/?model=BFS-U3-27S5C-C&vertical=machine+vision&segment=iis
   BFS_U3_27S5C(1936.0, 1464.0, UnitConversions.inchesToMeters(2.0 / 3.0), 0.00880, 0.00660);

   private final double imageWidthPixels;
   private final double imageHeightPixels;
   private final double cmosSensorFormat;
   private final double cmosSensorWidth;
   private final double cmosSensorHeight;

   BlackflyModelProperties(double imageWidthPixels,
                           double imageHeightPixels,
                           double cmosSensorFormat,
                           double cmosSensorWidth,
                           double cmosSensorHeight)
   {
      this.imageWidthPixels = imageWidthPixels;
      this.imageHeightPixels = imageHeightPixels;
      this.cmosSensorFormat = cmosSensorFormat;
      this.cmosSensorWidth = cmosSensorWidth;
      this.cmosSensorHeight = cmosSensorHeight;
   }

   public double getImageWidthPixels()
   {
      return imageWidthPixels;
   }

   public double getImageHeightPixels()
   {
      return imageHeightPixels;
   }

   public double getCmosSensorFormat()
   {
      return cmosSensorFormat;
   }

   public double getCmosSensorWidth()
   {
      return cmosSensorWidth;
   }

   public double getCmosSensorHeight()
   {
      return cmosSensorHeight;
   }
}
