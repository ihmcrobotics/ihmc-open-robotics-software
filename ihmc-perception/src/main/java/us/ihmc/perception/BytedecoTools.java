package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;
import us.ihmc.tools.thread.Activator;

import java.util.ArrayList;
import java.util.List;
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

   public static Activator loadGTSAMNativesOnAThread()
   {
      Activator nativesActivated = new Activator();
      ThreadTools.startAThread(BytedecoTools::loadGTSAMNatives, "GTSAM Loader");
      return nativesActivated;
   }

   public static void loadGTSAMNatives()
   {
      List<String> libraryFiles = new ArrayList<>();
      //      libraryFiles.add("libtbb.so");

      libraryFiles.add("libboost_filesystem.so");
      libraryFiles.add("libboost_chrono.so");
      libraryFiles.add("libboost_timer.so");
      libraryFiles.add("libboost_serialization.so");

      libraryFiles.add("libmetis-gtsam.so");
      libraryFiles.add("libgtsam.so");
      libraryFiles.add("libslam-wrapper.so");
      libraryFiles.add("libjniSlamWrapper.so");

      WorkspaceDirectory resourcesDirectory = new WorkspaceDirectory("ihmc-open-robotics-software", "ihmc-perception/src/slam-wrapper/resources");
      for (String libraryFile : libraryFiles)
      {
         System.load(new WorkspaceFile(resourcesDirectory, libraryFile).getFilePath().toAbsolutePath().normalize().toString());
      }
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
//      loadOpenCV();
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
