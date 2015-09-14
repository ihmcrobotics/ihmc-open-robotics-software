package us.ihmc.tools.inputDevices;

import java.io.File;

import org.apache.commons.lang3.SystemUtils;

import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class JInputLibraryLoader
{
   public static void loadLibraries()
   {
      String absolutePathToDirectory;
      if (SystemUtils.IS_OS_WINDOWS)
      {
         absolutePathToDirectory = NativeLibraryLoader.extractLibraries("", "jinput-raw", "jinput-raw_64", "jinput-dx8_64", "jinput-dx8", "jinput-wintab");
      }
      else if (SystemUtils.IS_OS_LINUX)
      {
         absolutePathToDirectory = NativeLibraryLoader.extractLibraries("", "jinput-linux64", "jinput-linux");
      }
      else if (SystemUtils.IS_OS_MAC)
      {
         absolutePathToDirectory = new File(NativeLibraryLoader.extractLibraryAbsolute("", "libjinput-osx.jnilib")).getParent();
      }
      else
      {
         return;
      }

      System.setProperty("net.java.games.input.librarypath", absolutePathToDirectory);
   }
}
