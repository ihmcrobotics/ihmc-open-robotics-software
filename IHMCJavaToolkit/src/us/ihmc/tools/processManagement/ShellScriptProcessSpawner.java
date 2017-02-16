package us.ihmc.tools.processManagement;

import java.io.File;

import org.apache.commons.lang3.ArrayUtils;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class ShellScriptProcessSpawner extends UnixProcessSpawner
{
   public ShellScriptProcessSpawner(boolean killChildProcessesOnShutdown)
   {
      super(killChildProcessesOnShutdown);
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
      String[] spawnString = new String[] {};

      spawnString = (String[]) ArrayUtils.add(spawnString, command);

      if (arguments != null && arguments.length > 0)
      {
         spawnString = (String[]) ArrayUtils.addAll(spawnString, arguments);
      }

      ProcessBuilder builder = new ProcessBuilder(spawnString);

      return spawn(command, spawnString, builder, outputLog, errorLog, exitListener);
   }
}
