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

import java.io.IOException;
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

   public static void loadMapsenseLibraries()
   {
      // We need to disable javacpp from trying to automatically load libraries.
      // Otherwise, it will try to load them by name when they aren't in the library path
      // (LD_LIBRARY_PATH on Linux).
      //
      // The approach taken here is to use System.load to load each library by explicit
      // absolute path on disk.
      System.setProperty("org.bytedeco.javacpp.loadlibraries", "false");

      List<String> libraryFiles = new ArrayList<>();

      // Load these libraries before laoding the wrapper JNI .so. Use ldd to list all necessary libs
      // and copy them.
      libraryFiles.add("libvisual-odometry.so");
      libraryFiles.add("libjniVisualOdometry.so");

      WorkspaceDirectory resourcesDirectory = new WorkspaceDirectory("ihmc-open-robotics-software", "ihmc-perception/src/mapsense-wrapper/resources");
      for (String libraryFile : libraryFiles)
      {
         WorkspaceFile file = new WorkspaceFile(resourcesDirectory, libraryFile);
         String filePath = file.getFilePath().toAbsolutePath().normalize().toString();
         System.out.println("Loading: " + filePath);
         System.load(filePath);
      }
   }

   public static String stringFromByteBuffer(BytePointer bytePointerWithString)
   {
      return bytePointerWithString.getString().trim();
   }
}
