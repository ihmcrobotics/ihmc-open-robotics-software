package us.ihmc.darpaRoboticsChallenge.diagnostics.logging;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.logging.FileHandler;
import java.util.logging.Filter;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

import org.apache.commons.io.output.TeeOutputStream;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class DiagnosticLoggerConfiguration
{
   public static void setupLogging(DoubleYoVariable yoTime, Class<?> clazz)
   {
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      Calendar calendar = Calendar.getInstance();
      String timestamp = dateFormat.format(calendar.getTime());

      Path diagnosticOutputDirectory = Paths.get(System.getProperty("user.home"), ".ihmc", "Diagnostic",
            timestamp + "_" + clazz.getSimpleName() + "_Outputs");
      try
      {
         Files.createDirectories(diagnosticOutputDirectory);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      DiagnosticLoggerFormatter formatter = new DiagnosticLoggerFormatter(yoTime);

      setupSystemOut(diagnosticOutputDirectory);
      setupLogFiles(diagnosticOutputDirectory, formatter);
   }

   private static void setupLogFiles(Path diagnosticOutputDirectory, DiagnosticLoggerFormatter formatter)
   {
      // Hack to get the root logger
      Logger rootLogger = Logger.getGlobal().getParent();

      Handler[] handlers = rootLogger.getHandlers();
      
      for (Handler handler : handlers)
         rootLogger.removeHandler(handler);

      rootLogger.addHandler(new DiagnosticLoggerSystemOutHandler(formatter, Level.INFO));
      rootLogger.addHandler(new DiagnosticLoggerSystemErrHandler(formatter, Level.INFO));
      
      try
      {
         String severeFileName = Paths.get(diagnosticOutputDirectory.toString(), "severe.log").toString();
         Handler severeFileHandler = new FileHandler(severeFileName);
         severeFileHandler.setFormatter(formatter);
         Filter severeFilter = new Filter()
         {
            @Override
            public boolean isLoggable(LogRecord record)
            {
               return record.getLevel() == Level.SEVERE;
            }
         };
         severeFileHandler.setFilter(severeFilter);
         rootLogger.addHandler(severeFileHandler);
      }
      catch (SecurityException | IOException e)
      {
         e.printStackTrace();
      }

      try
      {
         String warningFileName = Paths.get(diagnosticOutputDirectory.toString(), "warning.log").toString();
         Handler warningFileHandler = new FileHandler(warningFileName);
         warningFileHandler.setFormatter(formatter);
         Filter warningFilter = new Filter()
         {
            @Override
            public boolean isLoggable(LogRecord record)
            {
               return record.getLevel() == Level.WARNING;
            }
         };
         warningFileHandler.setFilter(warningFilter);
         rootLogger.addHandler(warningFileHandler);
      }
      catch (SecurityException | IOException e)
      {
         e.printStackTrace();
      }

      try
      {
         String allFileName = Paths.get(diagnosticOutputDirectory.toString(), "all.log").toString();
         Handler allFileHandler = new FileHandler(allFileName);
         allFileHandler.setFormatter(formatter);
         allFileHandler.setFilter(new Filter()
         {
            @Override
            public boolean isLoggable(LogRecord record)
            {
               return true;
            }
         });
         rootLogger.addHandler(allFileHandler);
      }
      catch (SecurityException | IOException e)
      {
         e.printStackTrace();
      }
   }

   private static void setupSystemOut(Path diagnosticOutputDirectory)
   {
      String consoleOutputPath = Paths.get(diagnosticOutputDirectory.toString(), "consoleOutput.log").toString();

      try
      {
         // we will want to print in standard "System.out" and in "consoleOutput.log"
         TeeOutputStream newOut = new TeeOutputStream(System.out, new FileOutputStream(consoleOutputPath));
         System.setOut(new PrintStream(newOut));
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

}
