package us.ihmc.perception.slamWrapper;

import us.ihmc.log.LogTools;
import us.ihmc.tools.nativelibraries.NativeLibraryDescription;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;
import us.ihmc.tools.nativelibraries.NativeLibraryWithDependencies;

public class SlamWrapperNativeLibrary implements NativeLibraryDescription
{
   @Override
   public String getPackage(OperatingSystem os, Architecture arch)
   {
      String archPackage = "";
      if (arch == Architecture.x64)
      {
         archPackage = switch (os)
               {
                  // TODO: Windows support
                  case LINUX64 -> "linux-x86_64";
                  default -> "unknown";
               };
      }

      return "slamWrapper." + archPackage;
   }

   @Override
   public NativeLibraryWithDependencies getLibraryWithDependencies(OperatingSystem os, Architecture arch)
   {
      switch (os)
      {
         // TODO: Windows support
         case LINUX64:
            return NativeLibraryWithDependencies.fromFilename("libjniSlamWrapper.so",
                                                              "libtbb.so",
                                                              "libboost_filesystem.so",
                                                              "libboost_chrono.so",
                                                              "libboost_timer.so",
                                                              "libboost_serialization.so",
                                                              "libmetis-gtsam.so",
                                                              "libgtsam.so",
                                                              "libslam-wrapper.so");
         default:
            break;
      }

      LogTools.warn("Unsupported platform: " + os.name() + "-" + arch.name());

      return null;
   }

   private static boolean loaded = false;

   public static boolean load()
   {
      if (!loaded)
      {
         SlamWrapperNativeLibrary slamWrapperNativeLibrary = new SlamWrapperNativeLibrary();
         loaded = NativeLibraryLoader.loadLibrary(slamWrapperNativeLibrary);
      }
      return loaded;
   }
}
