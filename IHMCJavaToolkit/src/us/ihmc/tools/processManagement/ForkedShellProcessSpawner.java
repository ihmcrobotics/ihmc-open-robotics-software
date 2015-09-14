package us.ihmc.tools.processManagement;

import java.io.File;
import java.util.Arrays;

import org.apache.commons.lang3.ArrayUtils;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class ForkedShellProcessSpawner extends UnixProcessSpawner
{
   private final String shellEnvironment;
   private final String[] shellEnvironmentArguments;

   public ForkedShellProcessSpawner(boolean killChildProcessesOnShutdown, String shellEnvironment, String[] shellEnvironmentArguments)
   {
      super(killChildProcessesOnShutdown);

      this.shellEnvironment = shellEnvironment;
      this.shellEnvironmentArguments = shellEnvironmentArguments;
   }

   public ForkedShellProcessSpawner(boolean shouldKillChildProcessesOnShutdown)
   {
      this(shouldKillChildProcessesOnShutdown, "/bin/sh", new String[]{ "-l", "-c" });
   }

   public Process spawn(String command)
   {
      return spawn(command, null, null, null, null);
   }

   public Process spawn(String command, String[] arguments)
   {
      return spawn(command, arguments, null, null, null);
   }

   public Process spawn(String command, String[] arguments, File outputLog, File errorLog, ExitListener exitListener)
   {
      if (shellEnvironment == null || shellEnvironment.isEmpty())
      {
         throw new RuntimeException("Shell environment set up incorrectly. Shell environment string: " + shellEnvironment);
      }

      if (shellEnvironmentArguments == null || shellEnvironmentArguments.length < 1)
      {
         throw new RuntimeException("Shell environment arguments seem bad. Arguments: " + Arrays.toString(shellEnvironmentArguments));
      }

      String[] spawnString = new String[] {};

      spawnString = (String[]) ArrayUtils.add(spawnString, shellEnvironment);
      spawnString = (String[]) ArrayUtils.addAll(spawnString, shellEnvironmentArguments);

      StringBuilder commandStringBuilder = new StringBuilder();
      commandStringBuilder.append(command);

      if (arguments == null) { arguments = new String[0]; }
      for (String argument : arguments) { commandStringBuilder.append(" ").append(argument); }

      spawnString = (String[]) ArrayUtils.add(spawnString, commandStringBuilder.toString().trim());
      ProcessBuilder builder = new ProcessBuilder(spawnString);

      return spawn(command, spawnString, builder, outputLog, errorLog, exitListener);
   }
}
