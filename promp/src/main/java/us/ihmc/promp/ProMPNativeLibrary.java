package us.ihmc.promp;

import us.ihmc.log.LogTools;
import us.ihmc.tools.nativelibraries.NativeLibraryDescription;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;
import us.ihmc.tools.nativelibraries.NativeLibraryWithDependencies;

public class ProMPNativeLibrary implements NativeLibraryDescription
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

      return "promp." + archPackage;
   }

   @Override
   public NativeLibraryWithDependencies getLibraryWithDependencies(OperatingSystem os, Architecture arch)
   {
      switch (os)
      {
         case WIN64:
            return NativeLibraryWithDependencies.fromFilename("jnipromp.dll", "promp.dll");
         case LINUX64:
            return NativeLibraryWithDependencies.fromFilename("libjnipromp.so", "libpromp.so");
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
         ProMPNativeLibrary prompLib = new ProMPNativeLibrary();
         loaded = NativeLibraryLoader.loadLibrary(prompLib);
      }
      return loaded;
   }
}
