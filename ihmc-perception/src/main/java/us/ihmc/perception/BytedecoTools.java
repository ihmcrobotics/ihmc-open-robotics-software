package us.ihmc.perception;

import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.Activator;

public class BytedecoTools
{
   public static Activator loadNativesOnAThread()
   {
      Activator nativesActivated = new Activator();
      ThreadTools.startAThread(() ->
      {
         loadNatives();
         nativesActivated.activate();
      }, "Bytedeco loader");
      return nativesActivated;
   }

   public static void loadNatives()
   {
      LogTools.info("Loading Bytedeco OpenCL...");
      Loader.load(OpenCL.class);
      LogTools.info("Bytedeco OpenCL loaded.");
      LogTools.info("Loading Bytedeco OpenCV...");
      Loader.load(opencv_core.class);
      LogTools.info("Bytedeco OpenCV loaded.");
   }
}
