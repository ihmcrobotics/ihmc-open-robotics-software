package us.ihmc.tools.processManagement;

import java.io.File;

import org.apache.commons.lang3.ArrayUtils;

/**
 * A lot of the magic in this class comes from inheriting the classpath of the current process before spawning
 * other processes.  Hence the lack of a main method here.  This should always be a field in some other class.
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class JavaProcessSpawner extends ProcessSpawner
{
   private final String jreHome = System.getProperty("java.home");
   private final String currentClassPath;
   private final String currentNativeLibraryPath;
   private final String ldLibraryPath;

   public JavaProcessSpawner(boolean killChildProcessesOnShutdown)
   {
      super(killChildProcessesOnShutdown);

      currentClassPath = System.getProperty("java.class.path");
      currentNativeLibraryPath = System.getProperty("java.library.path");
      ldLibraryPath = System.getenv("LD_LIBRARY_PATH");
   }

   public Process spawn(Class<?> mainClass)
   {
      return spawn(mainClass, null, null, null, null, null);
   }

   public Process spawn(Class<?> mainClass, String[] progArgs)
   {
      return spawn(mainClass, null, progArgs, null, null, null);
   }

   public Process spawn(Class<?> mainClass, String[] javaArgs, String[] programArgs)
   {
      return spawn(mainClass, javaArgs, programArgs, null, null, null);
   }

   public Process spawn(Class<?> mainClass, String[] javaArgs, String[] programArgs, File outputLog, File errorLog)
   {
      return spawn(mainClass, javaArgs, programArgs, outputLog, errorLog, null);
   }

   public Process spawn(Class<?> mainClass, String[] javaArgs, String[] programArgs, ExitListener exitListener)
   {
      return spawn(mainClass, javaArgs, programArgs, null, null, exitListener);
   }

   public Process spawn(Class<?> mainClass, ExitListener exitListener)
   {
      return spawn(mainClass, null, null, null, null, exitListener);
   }

   public Process spawn(Class<?> mainClass, String[] javaArgs, String[] programArgs, File outputLog, File errorLog, ExitListener exitListener)
   {
      String[] spawnString = setupSpawnString(mainClass, javaArgs, programArgs);
      ProcessBuilder builder = new ProcessBuilder(spawnString);

      if (isLinux() && (ldLibraryPath != null))
      {
         builder.environment().put("LD_LIBRARY_PATH", ldLibraryPath);
      }

      return spawn(mainClass.getSimpleName(), spawnString, builder, outputLog, errorLog, exitListener);
   }

   private String[] setupSpawnString(Class<?> mainClass, String[] javaArgs, String[] programArgs)
   {
      String[] spawnString = new String[] {jreHome + "/bin/java"};

      if (javaArgs != null)
      {
         spawnString = (String[]) ArrayUtils.addAll(spawnString, javaArgs);
      }

      String fqClassName = mainClass.getCanonicalName();
      String[] cp = new String[] { "-Djava.library.path=" + currentNativeLibraryPath, "-cp", currentClassPath, fqClassName };
      spawnString = (String[]) ArrayUtils.addAll(spawnString, cp);

      if (programArgs != null)
      {
         spawnString = (String[]) ArrayUtils.addAll(spawnString, programArgs);
      }

      return spawnString;
   }

   private boolean isLinux()
   {
      String os = System.getProperty("os.name").toLowerCase();

      return (os.contains("nix") || os.contains("nux") || os.contains("aix"));
   }

   public void prettyPrintClassPath()
   {
      String[] paths = currentClassPath.split(":");
      for (String s : paths)
      {
         System.out.println(s);
      }
   }

   public void prettyPrintNativeLibraryPath()
   {
      String[] paths = currentNativeLibraryPath.split(":");
      for (String s : paths)
      {
         System.out.println(s);
      }
   }

   @Override public void kill(Process process)
   {
      process.destroy();
   }
}
