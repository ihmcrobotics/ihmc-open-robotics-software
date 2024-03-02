package us.ihmc.mpc.mpcWrapper.presets;

import us.ihmc.log.LogTools;
import us.ihmc.tools.nativelibraries.NativeLibraryDescription;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;
import us.ihmc.tools.nativelibraries.NativeLibraryWithDependencies;

public class MPCWrapperNativeLibrary implements NativeLibraryDescription
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
            default -> "unknown";
         };
      }

      return "mpcWrapper." + archPackage;
   }

   @Override
   public NativeLibraryWithDependencies getLibraryWithDependencies(OperatingSystem os, Architecture arch)
   {
      switch (os)
      {
         case LINUX64:
            return NativeLibraryWithDependencies.fromFilename("libjniMPCWrapper.so", "libblasfeo.so", "libhpipm.so", "libmpc-wrapper.so");
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
         MPCWrapperNativeLibrary mpcWrapperNativeLibrary = new MPCWrapperNativeLibrary();
         loaded = NativeLibraryLoader.loadLibrary(mpcWrapperNativeLibrary);
      }
      return loaded;
   }
}
