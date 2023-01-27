package us.ihmc.perception;

import org.junit.jupiter.api.Test;
import us.ihmc.perception.zedDriver.ZEDOpenDriver;

public class ZEDOpenDriverTest
{
   @Test
   public static void testLoadingImageDimensions()
   {
      BytedecoTools.loadZEDDriverNative();
      int[] dims;
      try (ZEDOpenDriver.ZEDOpenDriverExternal zedDriver = new ZEDOpenDriver.ZEDOpenDriverExternal())
      {
         dims = new int[] {0, 0};
         zedDriver.getFrameDimensions(dims);
      }

      assert(dims[0] != 0);
      assert(dims[1] != 0);
   }

}
