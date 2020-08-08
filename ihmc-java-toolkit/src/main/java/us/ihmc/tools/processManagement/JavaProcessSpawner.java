package us.ihmc.tools.processManagement;

import java.io.File;
import java.io.PrintStream;

import org.apache.commons.lang3.SystemUtils;

/**
 * A lot of the magic in this class comes from inheriting the classpath of the current process before spawning
 * other processes.  Hence the lack of a main method here.  This should always be a field in some other class.
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class JavaProcessSpawner extends ProcessSpawner
{
   private final String javaHome = System.getProperty("java.home");
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

   public Process spawn(Class<?> mainClass, String[] javaArgs, String[] programArgs, File outputLog, File errorLog, ExitListener exitListener)
   {
      return spawn(mainClass, javaArgs, programArgs, outputLog, errorLog, null, null, exitListener);
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
      return spawn(mainClass,
                   javaArgs,
                   programArgs,
                   outputFile,
                   errorFile,
                   outputStream,
                   errorStream,
                   ProcessSpawner.defaultPrintingPrefix(mainClass.getSimpleName()),
                   exitListener);
   }

   public Process spawn(Class<?> mainClass,
                        String[] javaArgs,
                        String[] programArgs,
                        File outputFile,
                        File errorFile,
                        PrintStream outputStream,
                        PrintStream errorStream,
                        String processPrintingPrefix,
                        ExitListener exitListener)
   {
      String[] spawnString = ProcessTools.constructJavaProcessCommand(javaHome,
                                                                      currentNativeLibraryPath,
                                                                      useEnvironmentForClasspath ? null : currentClassPath,
                                                                      mainClass,
                                                                      javaArgs,
                                                                      programArgs);
      ProcessBuilder builder = new ProcessBuilder(spawnString);

      if (SystemUtils.IS_OS_UNIX && (ldLibraryPath != null))
      {
         builder.environment().put("LD_LIBRARY_PATH", ldLibraryPath);
      }

      if (useEnvironmentForClasspath)
      {
         builder.environment().put("CLASSPATH", currentClassPath);
      }

      return spawn(mainClass.getSimpleName(),
                   spawnString,
                   builder,
                   outputFile,
                   errorFile,
                   outputStream,
                   errorStream,
                   processPrintingPrefix,
                   exitListener);
   }

   public void prettyPrintClassPath()
   {
      String[] paths = currentClassPath.split(File.pathSeparator);
      for (String path : paths)
      {
         System.out.println(path);
      }
   }

   public void prettyPrintNativeLibraryPath()
   {
      String[] paths = currentNativeLibraryPath.split(File.pathSeparator);
      for (String path : paths)
      {
         System.out.println(path);
      }
   }

   @Override
   public void kill(Process process)
   {
      process.destroy();
   }
}
