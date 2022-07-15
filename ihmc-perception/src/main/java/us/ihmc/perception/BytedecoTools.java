package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.Activator;

import java.util.concurrent.ConcurrentLinkedQueue;

public class BytedecoTools
{
   public static Activator loadNativesOnAThread(Class<?>... classes)
   {
      Activator nativesActivated = new Activator();
      ThreadTools.startAThread(() ->
      {
         ConcurrentLinkedQueue<Notification> notifications = new ConcurrentLinkedQueue<>();
         for (Class<?> clazz : classes)
         {
            Notification loadedNotification = new Notification();
            notifications.add(loadedNotification);
            ThreadTools.startAThread(() ->
            {
               LogTools.info("Loading Bytedeco {}...", clazz.getSimpleName());
               Loader.load(clazz);
               LogTools.info("Bytedeco {} loaded.", clazz.getSimpleName());
               loadedNotification.set();
            }, "Loading" + clazz.getSimpleName());
         }

         while (!notifications.isEmpty())
         {
            notifications.poll().blockingPoll();
         }

         nativesActivated.activate();
      }, "Bytedeco loader");
      return nativesActivated;
   }

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

   public static Activator loadOpenCVNativesOnAThread()
   {
      Activator nativesActivated = new Activator();
      ThreadTools.startAThread(() ->
      {
         loadOpenCV();
         nativesActivated.activate();
      }, "Bytedeco loader");
      return nativesActivated;
   }

   public static void loadNatives()
   {
      loadOpenCL();
      loadOpenCV();
   }

   public static void loadOpenCL()
   {
      LogTools.info("Loading Bytedeco OpenCL...");
      Loader.load(OpenCL.class);
      LogTools.info("Bytedeco OpenCL loaded.");
   }

   public static void loadOpenCV()
   {
      LogTools.info("Loading Bytedeco OpenCV...");
      Loader.load(opencv_core.class);
      LogTools.info("Bytedeco OpenCV loaded.");
   }

   public static String stringFromByteBuffer(BytePointer bytePointerWithString)
   {
      return bytePointerWithString.getString().trim();
   }
}
