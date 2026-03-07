package us.ihmc.perception.demo;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.opencv.global.opencv_highgui;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.RawImage;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;

import static us.ihmc.zed.global.zed.*;

public class DemoZEDImageSensor
{
   private static final boolean running = true;

   public DemoZEDImageSensor(int cameraID, ZEDModelData zedModelData, int inputType, int depthMode, int resolution, int fps) throws InterruptedException
   {
      ZEDImageSensor zedImageSensor = new ZEDImageSensor(cameraID, zedModelData, inputType, depthMode, resolution, fps);

      zedImageSensor.run(true);

      while (!zedImageSensor.isSensorRunning())

      while (running)
      {
         zedImageSensor.waitForGrab();

         RawImage image = zedImageSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);

         Mat imageMat = image.getCpuImageMat();

         opencv_highgui.imshow("Left", imageMat);
         opencv_highgui.waitKey(1);

         image.release();

         imageMat.close();
         Runtime.getRuntime().addShutdownHook(new Thread(zedImageSensor::close));
      }
   }

   public static void main(String[] args) throws InterruptedException
   {
      int fps;
      if (args[0].equals("1"))
         fps = 30;
      else
         fps = 60;

      System.out.println("Running with " + fps + " fps");
      new DemoZEDImageSensor(0, ZEDModelData.ZED_X_MINI, SL_INPUT_TYPE_GMSL, SL_DEPTH_MODE_NEURAL, SL_RESOLUTION_SVGA, fps);
   }
}
