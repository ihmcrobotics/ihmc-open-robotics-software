package us.ihmc.robotDataCommunication.logger.util;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.MalformedURLException;
import java.net.URL;


public class ExternalProgramHelpers
{
   public static String extractExternalProgram(URL url)
   {
     
      if(url.getProtocol().equals("file"))
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
         while((readbytes = inStream.read(buffer)) != -1)
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
   
   public static String getOS()
   {
      String os = System.getProperty("os.name");
      
      if("mac os x".equals(os.toLowerCase()))
      {
         return "mac";
      }
      
      if(os != null && os.toLowerCase().indexOf("windows") != -1)
      {
         return "windows";
      }
      
      if(os != null && os.toLowerCase().indexOf("linux") != -1)
      {
         return "linux";
      }
      
      throw new RuntimeException("Unsupported OS " + os);
   }
   
   public static String getExecutableExtension()
   {
      String os = System.getProperty("os.name");
      
      if(os != null && os.indexOf("windows") != -1)
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
