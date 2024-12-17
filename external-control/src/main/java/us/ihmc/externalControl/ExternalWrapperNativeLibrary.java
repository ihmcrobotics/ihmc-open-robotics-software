package us.ihmc.externalControl;

import us.ihmc.log.LogTools;
import us.ihmc.tools.nativelibraries.NativeLibraryDescription;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;
import us.ihmc.tools.nativelibraries.NativeLibraryWithDependencies;

public class ExternalWrapperNativeLibrary implements NativeLibraryDescription
{
   @Override
   public String getPackage(OperatingSystem operatingSystem, Architecture architecture)
   {
      String architecturePackage = "";
      if (architecture == Architecture.x64)
      {
         architecturePackage = switch (operatingSystem)
         {
            case LINUX64 -> "linux-x86_64";
            default -> "unknown";
         };
      }

      return "externalWrapper." + architecturePackage;
   }

   @Override
   public NativeLibraryWithDependencies getLibraryWithDependencies(OperatingSystem operatingSystem, Architecture architecture)
   {
      switch (operatingSystem)
      {
         case LINUX64:
            return NativeLibraryWithDependencies.fromFilename("jniExternalWrapper.so",
                                                              "libexternal-wrapper.so");
         default:
            break;
      }

      LogTools.warn("Unsupported platform: " + operatingSystem.name() + "-" + architecture.name());
      return null;
   }

   private static boolean loaded = false;

   public static boolean load()
   {
      if (!loaded)
      {
         ExternalWrapperNativeLibrary nativeLibrary = new ExternalWrapperNativeLibrary();
         loaded = NativeLibraryLoader.loadLibrary(nativeLibrary);
      }
      return loaded;
   }
}
