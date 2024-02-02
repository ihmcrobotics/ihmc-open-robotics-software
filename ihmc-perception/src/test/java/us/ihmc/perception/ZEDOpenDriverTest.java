package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.perception.zedDriver.ZEDOpenDriver;
import us.ihmc.perception.zedDriver.ZedDriverNativeLibrary;

public class ZEDOpenDriverTest
{
   @Test
   public void testLoadingImageDimensions()
   {
      ZedDriverNativeLibrary.load();
      int[] dims;
      try (ZEDOpenDriver.ZEDOpenDriverExternal zedDriver = new ZEDOpenDriver.ZEDOpenDriverExternal(720, 60))
      {
         dims = new int[] {0, 0};
         zedDriver.getFrameDimensions(dims);
      }

      LogTools.info("Dimensions: " + dims[0] + ", " + dims[1]);

      assert (dims[0] != 0);
      assert (dims[1] != 0);
   }

   @Test
   public void testLoadingImage()
   {
      ZedDriverNativeLibrary.load();
      int[] dims;
      ZEDOpenDriver.ZEDOpenDriverExternal zedDriver = new ZEDOpenDriver.ZEDOpenDriverExternal(1080, 30);
      dims = new int[] {0, 0, 0};
      zedDriver.getFrameDimensions(dims);

      byte[] image = new byte[dims[0] * dims[1] * dims[2]];
      zedDriver.getFrameStereoYUVExternal(image, dims);

      BytePointer bytePointer = new BytePointer(image);

      Mat mat = new Mat(dims[0], dims[1], opencv_core.CV_8UC2, bytePointer);

      LogTools.info("Dimensions: " + dims[0] + ", " + dims[1]);

      opencv_imgproc.cvtColor(mat, mat, opencv_imgproc.COLOR_YUV2BGR_YUYV);
      PerceptionDebugTools.display("Image", mat, 0);

      assert (dims[0] == mat.rows());
      assert (dims[1] == mat.cols());
      assert (dims[2] == 2);
   }
}
