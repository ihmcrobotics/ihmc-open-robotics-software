package us.ihmc.perception.visualOdometry;

import us.ihmc.log.LogTools;
import us.ihmc.tools.nativelibraries.NativeLibraryDescription;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;
import us.ihmc.tools.nativelibraries.NativeLibraryWithDependencies;

public class VisualOdometryNativeLibrary implements NativeLibraryDescription
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

      return "visualOdometry." + archPackage;
   }

   @Override
   public NativeLibraryWithDependencies getLibraryWithDependencies(OperatingSystem os, Architecture arch)
   {
      switch (os)
      {
         // TODO: Windows support
         case LINUX64:

            return NativeLibraryWithDependencies.fromFilename("libvisual-odometry.so",
                                                              "libjniVisualOdometry.so");
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
         VisualOdometryNativeLibrary visualOdometryNativeLibrary = new VisualOdometryNativeLibrary();
         loaded = NativeLibraryLoader.loadLibrary(visualOdometryNativeLibrary);
      }
      return loaded;
   }
}
