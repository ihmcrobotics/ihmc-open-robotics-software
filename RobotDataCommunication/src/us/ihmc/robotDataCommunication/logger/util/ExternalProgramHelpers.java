package us.ihmc.robotDataCommunication.logger.util;

public class ExternalProgramHelpers
{
   public static String getOS()
   {
      String os = System.getProperty("os.name");
      
      if("mac os x".equals(os))
      {
         return "mac";
      }
      
      if(os != null && os.indexOf("windows") != -1)
      {
         return "windows";
      }
      
      if(os != null && os.indexOf("linux") != -1)
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
}
