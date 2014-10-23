package us.ihmc.robotDataCommunication.logger.util;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.MalformedURLException;
import java.net.URL;

import us.ihmc.utilities.operatingSystem.OperatingSystem;
import us.ihmc.utilities.operatingSystem.OperatingSystemTools;

public class ExternalProgramHelpers
{
   private static final OperatingSystem currentOS = OperatingSystemTools.getOperatingSystem();

   public static String extractExternalProgram(URL url)
   {
      if (url.getProtocol().equals("file"))
      {
         return url.getFile();
      }

      try
      {
         File output = File.createTempFile("external", getExecutableExtension());
         output.deleteOnExit();
         output.setExecutable(true);

         BufferedInputStream inStream = new BufferedInputStream(url.openStream());
         OutputStream outStream = new FileOutputStream(output);
         byte[] buffer = new byte[1024];
         int readbytes;
         while ((readbytes = inStream.read(buffer)) != -1)
         {
            outStream.write(buffer, 0, readbytes);
         }

         outStream.close();
         inStream.close();
         return output.getAbsolutePath();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static OperatingSystem getOS()
   {
      if (currentOS == null)
         throw new RuntimeException("Unsupported OS " + OperatingSystemTools.getOperatingSystemName());
      
      return currentOS;
   }

   public static String getOSNameAsString()
   {
      return getOS().toString().toLowerCase();
   }

   public static String getExecutableExtension()
   {
      if (currentOS != null && currentOS == OperatingSystem.WINDOWS)
      {
         return ".exe";
      }

      return "";
   }

   public static void main(String[] args) throws MalformedURLException
   {
      System.out.println(extractExternalProgram(new URL("file:///home/jesper/bin/ffmpeg")));
   }
}
