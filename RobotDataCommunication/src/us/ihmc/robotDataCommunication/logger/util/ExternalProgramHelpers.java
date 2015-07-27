package us.ihmc.robotDataCommunication.logger.util;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.MalformedURLException;
import java.net.URL;

import org.apache.commons.lang3.SystemUtils;

public class ExternalProgramHelpers
{

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

   public static String getOSNameAsString()
   {
      if(SystemUtils.IS_OS_WINDOWS)
      {
         return "windows";
      }
      else if (SystemUtils.IS_OS_LINUX)
      {
         return "linux";
      }
      else if (SystemUtils.IS_OS_MAC)
      {
         return "mac";
      }
      else
      {
         throw new RuntimeException("Unsupported OS" + SystemUtils.OS_NAME);
      }
   }

   public static String getExecutableExtension()
   {
      if (SystemUtils.IS_OS_WINDOWS)
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
