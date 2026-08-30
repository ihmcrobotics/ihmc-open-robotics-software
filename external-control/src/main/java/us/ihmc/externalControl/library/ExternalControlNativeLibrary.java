package us.ihmc.externalControl.library;

import us.ihmc.externalControl.global.ExternalControlWrapper;
import us.ihmc.tools.nativelibraries.NativeLibraryDescription;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;
import us.ihmc.tools.nativelibraries.NativeLibraryWithDependencies;

public class ExternalControlNativeLibrary implements NativeLibraryDescription
{
   @Override
   public String getPackage(OperatingSystem os, Architecture arch)
   {
      String archPackage = "";
      if (arch == Architecture.x64)
      {
         archPackage = switch (os)
         {
            case WIN64 -> "windows-x86_64";
            case LINUX64 -> "linux-x86_64";
            case MACOSX64 -> "macos-x86_64";
         };
      }
      else if (arch == Architecture.arm64)
      {
         archPackage = switch (os)
         {
            case WIN64 -> "windows-arm64";
            case LINUX64 -> "linux-arm64";
            case MACOSX64 -> "macos-arm64";
         };
      }

      return "externalControl.native." + archPackage;
   }

   @Override
   public NativeLibraryWithDependencies getLibraryWithDependencies(OperatingSystem os, Architecture arch)
   {
      if (os == OperatingSystem.LINUX64 && arch == Architecture.x64)
         return NativeLibraryWithDependencies.fromFilename("libjniExternalControlWrapper.so", "libexternal-control.so", "libzmq.so.5");//, "libzmq.so.5");

      throw new RuntimeException("Unsupported platform: " + os.name() + "-" + arch.name());
   }

   private static boolean loaded = false;

   public static boolean load()
   {
      if (!loaded)
      {
         ExternalControlNativeLibrary lib = new ExternalControlNativeLibrary();
         loaded = NativeLibraryLoader.loadLibrary(lib);
      }
      return loaded;
   }
}