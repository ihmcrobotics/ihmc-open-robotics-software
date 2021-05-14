package us.ihmc.tools.processManagement;

import org.apache.commons.io.output.TeeOutputStream;
import org.apache.commons.lang3.ArrayUtils;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;

import java.io.IOException;
import java.io.PrintStream;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;

/**
 * Print what is starting as it starts.
 * <p>
 * Only prefix output when more than one process is spawned.
 * <p>
 * Use case for use in a main class:
 * - preventing fork recursion
 * - logging to timestamped .ihmc/logs file
 * - starting multiple processes with runnables
 *
 * start method must be called after all processes are added via runOrRegister
 */
public class JavaProcessManager
{
   private static final String FORKED_PROCESS_INDEX = "forked.process.index";
   private static final String forkedProcessIndexProperty = System.getProperty(FORKED_PROCESS_INDEX);
   private static final boolean isSpawnerProcess = forkedProcessIndexProperty == null;
   private static final int forkedProcessIndex = isSpawnerProcess ? -1 : Integer.parseInt(forkedProcessIndexProperty);

   private final ArrayList<String> processNames = new ArrayList<>();

   public static ArrayList<Process> teeToLogFile(Class<?> mainClass)
   {
      JavaProcessManager manager = new JavaProcessManager();
      manager.runOrRegister(mainClass);
      return manager.spawnProcesses(mainClass, null);
   }

   public JavaProcessManager()
   {

   }

   public ArrayList<Process> spawnProcesses(Class<?> mainClass, String[] args)
   {
      if (isSpawnerProcess) // fork processes
      {
         ArrayList<Process> processList = new ArrayList<>();

         try
         {
            JavaProcessSpawner spawner = new JavaProcessSpawner(true);
            String[] parentJVMProperties = ProcessTools.getCurrentJVMProperties();

            for (int i = 0; i < processNames.size(); i++)
            {
               DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmssSSS");
               Calendar calendar = Calendar.getInstance();
               String timestampOfCreation = dateFormat.format(calendar.getTime());
               Path logFilePath = Paths.get(System.getProperty("user.home"),
                                            ".ihmc",
                                            "logs",
                                            timestampOfCreation + "_" + processNames.get(i) + "Log.txt");
               String[] jvmProperties = ArrayUtils.add(parentJVMProperties, "-D" + FORKED_PROCESS_INDEX + "=" + i);

               TeeOutputStream outputTee = new TeeOutputStream(System.out, ProcessTools.createJansiFilteredStream(logFilePath));
               TeeOutputStream errorTee = new TeeOutputStream(System.err, ProcessTools.createJansiFilteredStream(logFilePath));
               PrintStream outputStream = new PrintStream(outputTee);
               PrintStream errorStream = new PrintStream(errorTee);

               LogTools.info("Spawning process {} teed to {}", processNames.get(i), logFilePath);

               String prefix = processNames.size() > 1 ? ProcessSpawner.defaultPrintingPrefix(processNames.get(i)) : null;

               processList.add(spawner.spawn(mainClass, jvmProperties, args, null, null, outputStream, errorStream, prefix, null));
            }
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }

         return processList;
      }

      return null;
   }

   public boolean isSpawnerProcess()
   {
      return isSpawnerProcess;
   }

   public void runOrRegister(Class<?> mainClass)
   {
      runOrRegister(mainClass.getSimpleName(), () -> ExceptionTools.handle(mainClass::newInstance, DefaultExceptionHandler.PRINT_STACKTRACE));
   }

   public void runOrRegister(String name, Runnable runnable)
   {
      if (!isSpawnerProcess && forkedProcessIndex == processNames.size())
      {
         LogTools.info("Starting " + name);
         runnable.run();
      }

      processNames.add(name);
   }
}
