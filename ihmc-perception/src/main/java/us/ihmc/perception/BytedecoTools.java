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

   public static Activator loadGTSAMNativesOnAThread()
   {
      Activator nativesActivated = new Activator();
      ThreadTools.startAThread(BytedecoTools::loadGTSAMNatives, "GTSAM Loader");
      return nativesActivated;
   }

   public static void loadGTSAMNatives()
   {
      // We need to disable javacpp from trying to automatically load libraries.
      // Otherwise, it will try to load them by name when they aren't in the library path
      // (LD_LIBRARY_PATH on Linux).
      //
      // The approach taken here is to use System.load to load each library by explicit
      // absolute path on disk.
      System.setProperty("org.bytedeco.javacpp.loadlibraries", "false");

      List<String> libraryFiles = new ArrayList<>();
      libraryFiles.add("libtbb.so");

      libraryFiles.add("libboost_thread.so");
      libraryFiles.add("libboost_stacktrace_addr2line.so");
      libraryFiles.add("libboost_filesystem.so");
      libraryFiles.add("libboost_stacktrace_basic.so");
      libraryFiles.add("libboost_date_time.so");
      libraryFiles.add("libboost_context.so");
      libraryFiles.add("libboost_math_c99l.so");
      libraryFiles.add("libboost_program_options.so");
      libraryFiles.add("libboost_serialization.so");
      libraryFiles.add("libboost_stacktrace_noop.so");
      libraryFiles.add("libboost_prg_exec_monitor.so");
      libraryFiles.add("libboost_coroutine.so");
      libraryFiles.add("libboost_chrono.so");
      libraryFiles.add("libboost_timer.so");
      libraryFiles.add("libboost_stacktrace_backtrace.so");
      libraryFiles.add("libboost_math_c99f.so");
      libraryFiles.add("libboost_system.so");
      libraryFiles.add("libboost_wserialization.so");
      libraryFiles.add("libboost_atomic.so");
      libraryFiles.add("libboost_math_c99.so");
      libraryFiles.add("libboost_fiber.so");
      libraryFiles.add("libboost_unit_test_framework.so");
      libraryFiles.add("libboost_math_tr1.so");
      libraryFiles.add("libboost_container.so");
      libraryFiles.add("libboost_random.so");
      libraryFiles.add("libboost_type_erasure.so");
      libraryFiles.add("libboost_math_tr1f.so");
      libraryFiles.add("libboost_math_tr1l.so");
      libraryFiles.add("libboost_wave.so");

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
