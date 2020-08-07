package us.ihmc.tools.processManagement;

import java.io.File;
import java.io.PrintStream;

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
   private final String currentClassPath = System.getProperty("java.class.path");
   private final String currentNativeLibraryPath = System.getProperty("java.library.path");
   private final String ldLibraryPath = System.getenv("LD_LIBRARY_PATH");
   private final boolean useEnvironmentForClasspath;

   public JavaProcessSpawner(boolean killChildProcessesOnShutdown, boolean useEnvironmentForClasspath)
   {
      super(killChildProcessesOnShutdown);

      this.useEnvironmentForClasspath = useEnvironmentForClasspath;
   }

   public JavaProcessSpawner(boolean killChildProcessesOnShutdown)
   {
      this(killChildProcessesOnShutdown, false);
   }

   public Process spawn(Class<?> mainClass)
   {
      return spawn(mainClass, null, null, null, null, null, null, null);
   }

   public Process spawn(Class<?> mainClass, String[] progArgs)
   {
      return spawn(mainClass, null, progArgs, null, null, null, null, null);
   }

   public Process spawn(Class<?> mainClass, String[] javaArgs, String[] programArgs)
   {
      return spawn(mainClass, javaArgs, programArgs, null, null, null, null, null);
   }

   public Process spawn(Class<?> mainClass, String[] javaArgs, String[] programArgs, File outputLog, File errorLog)
   {
      return spawn(mainClass, javaArgs, programArgs, outputLog, errorLog, null, null, null);
   }

   public Process spawn(Class<?> mainClass, String[] javaArgs, String[] programArgs, PrintStream outputStream, PrintStream errorStream)
   {
      return spawn(mainClass, javaArgs, programArgs, null, null, outputStream, errorStream, null);
   }

   public Process spawn(Class<?> mainClass, String[] javaArgs, String[] programArgs, ExitListener exitListener)
   {
      return spawn(mainClass, javaArgs, programArgs, null, null, null, null, exitListener);
   }

   public Process spawn(Class<?> mainClass, ExitListener exitListener)
   {
      return spawn(mainClass, null, null, null, null, null, null, exitListener);
   }

   public Process spawn(Class<?> mainClass,
                        String[] javaArgs,
                        String[] programArgs,
                        File outputFile,
                        File errorFile,
                        PrintStream outputStream,
                        PrintStream errorStream,
                        ExitListener exitListener)
   {
      String[] spawnString = setupSpawnString(mainClass, javaArgs, programArgs);
      ProcessBuilder builder = new ProcessBuilder(spawnString);

      if (isLinux() && (ldLibraryPath != null))
      {
         builder.environment().put("LD_LIBRARY_PATH", ldLibraryPath);
      }

      if(useEnvironmentForClasspath)
      {
         builder.environment().put("CLASSPATH", currentClassPath);
      }

      return spawn(mainClass.getSimpleName(), spawnString, builder, outputFile, errorFile, outputStream, errorStream, exitListener);
   }

   private String[] setupSpawnString(Class<?> mainClass, String[] javaArgs, String[] programArgs)
   {
      String[] spawnString = new String[] {jreHome + "/bin/java"};

      if (javaArgs != null)
      {
         spawnString = ArrayUtils.addAll(spawnString, javaArgs);
      }

      String fqClassName = mainClass.getCanonicalName();
      String[] cp = new String[] { "-Djava.library.path=" + currentNativeLibraryPath};

      if(!useEnvironmentForClasspath)
      {
         cp = ArrayUtils.addAll(cp, "-cp", currentClassPath);
      }

      spawnString = ArrayUtils.addAll(spawnString, cp);
      spawnString = ArrayUtils.addAll(spawnString, fqClassName);

      if (programArgs != null)
      {
         spawnString = ArrayUtils.addAll(spawnString, programArgs);
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
      String[] paths = currentClassPath.split(File.pathSeparator);
      for (String s : paths)
      {
         System.out.println(s);
      }
   }

   public void prettyPrintNativeLibraryPath()
   {
      String[] paths = currentNativeLibraryPath.split(File.pathSeparator);
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
