package us.ihmc.robotDataCommunication.logger.util;

public class CookieJar implements ExternalProgram
{
   private static final String cookieJarSH = ExternalProgramHelpers.extractExternalProgram(CookieJar.class.getResource("bin/" + ExternalProgramHelpers.getOSNameAsString()
         + "/cookiejar.sh"));

   private String host;
   private String remoteDirectory;
   private String directory;
   private String user;

   public CookieJar()
   {
   }

   public void setHost(String host)
   {
      this.host = host;
   }

   public void setRemoteDirectory(String remoteDirectory)
   {
      this.remoteDirectory = remoteDirectory;
   }

   public void setDirectory(String directory)
   {
      this.directory = directory;
   }

   public void setUser(String user)
   {
      this.user = user;
   }

   private void appendCmdOption(StringBuilder cmd, String... args)
   {
      for (String arg : args)
      {
         cmd.append(" ");
         cmd.append(arg);
      }
   }

   public String getCommandLine()
   {
      StringBuilder cmd = new StringBuilder();

      cmd.append(cookieJarSH);

      if (host != null)
      {
         appendCmdOption(cmd, "-h", host);
      }
      if (remoteDirectory != null)
      {
         appendCmdOption(cmd, "-r", remoteDirectory);
      }
      if (directory != null)
      {
         appendCmdOption(cmd, "-d", directory);
      }
      if (user != null)
      {
         appendCmdOption(cmd, "-u", user);
      }
      return cmd.toString();
   }

}
