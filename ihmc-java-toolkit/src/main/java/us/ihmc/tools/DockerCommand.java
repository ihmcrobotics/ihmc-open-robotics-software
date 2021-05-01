package us.ihmc.tools;

import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspacePathTools;

import java.io.IOException;
import java.nio.file.Path;

/**
 * This class is designed to manage a Docker container
 * via a bash script in the resources directory. See
 * usages for examples.
 */
public class DockerCommand
{
   private ProcessBuilder processBuilder;
   private Process process;

   public DockerCommand(String projectName, String scriptPath)
   {
      Path script = WorkspacePathTools.handleWorkingDirectoryFuzziness(projectName)
                                      .resolve(scriptPath)
                                      .toAbsolutePath()
                                      .normalize();

      LogTools.info("Command: {}", script);

      processBuilder = new ProcessBuilder("/bin/bash", script.toString());
      processBuilder.redirectOutput(ProcessBuilder.Redirect.INHERIT);
      processBuilder.redirectError(ProcessBuilder.Redirect.INHERIT);
   }

   public void start()
   {
      try
      {
         process = processBuilder.start();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      Runtime.getRuntime().addShutdownHook(new Thread(this::shutdown));
   }

   public void waitFor()
   {
      try
      {
         process.waitFor();
      }
      catch (InterruptedException interruptedException)
      {
         interruptedException.printStackTrace();
      }
   }

   public void shutdown()
   {
      if (process != null && process.isAlive())
      {
         process.destroy();
         LogTools.info("Destroying process...");
         try
         {
            process.waitFor();
         }
         catch (InterruptedException interruptedException)
         {
            interruptedException.printStackTrace();
         }
         LogTools.info("Done.");
      }
   }
}
