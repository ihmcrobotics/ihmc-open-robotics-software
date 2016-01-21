package us.ihmc.wholeBodyController.diagnostics.logging;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.net.InetAddress;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.logging.FileHandler;
import java.util.logging.Filter;
import java.util.logging.Formatter;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

import org.apache.commons.io.output.TeeOutputStream;
import org.apache.commons.net.ntp.NTPUDPClient;
import org.apache.commons.net.ntp.TimeInfo;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class DiagnosticLoggerConfiguration
{
   public static void setupLogging(DoubleYoVariable yoTime, Class<?> clazz, String robotName)
   {
      setupLogging(yoTime, clazz, robotName, false);
   }

   public static void setupLogging(DoubleYoVariable yoTime, Class<?> clazz, String robotName, boolean useInternetDate)
   {
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");

      Date time;

      if (!useInternetDate)
      {
         time = getLocalTime();
      }
      else
      {
         time = getTimeFromServer();
         if (time == null)
            time = getLocalTime();
      }

      String timestamp = dateFormat.format(time);

      Path diagnosticOutputDirectory = Paths.get(System.getProperty("user.home"), ".ihmc", "Diagnostic",
            timestamp + "_" + robotName + "_" + clazz.getSimpleName() + "_Outputs");
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

   private static Date getLocalTime()
   {
      Calendar calendar = Calendar.getInstance();
      return calendar.getTime();
   }

   public static void main(String[] args)
   {
      System.out.println(getTimeFromServer());
   }
   
   private static Date getTimeFromServer()
   {
      InetAddress inetAddress;
      try
      {
         String TIME_SERVER = "98.175.203.200";
//         String TIME_SERVER = "nist1-macon.macon.ga.us"; // Does not work on Atlas :'(
         NTPUDPClient timeClient = new NTPUDPClient();
         inetAddress = InetAddress.getByName(TIME_SERVER);
         TimeInfo timeInfo = timeClient.getTime(inetAddress);
         timeInfo.computeDetails();
         long actualTime = timeInfo.getReturnTime() + timeInfo.getOffset();
         System.out.println(timeInfo.getOffset());
         Date time = new Date(actualTime);
         return time;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
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
      
      Filter severeFilter = new Filter()
      {
         @Override
         public boolean isLoggable(LogRecord record)
         {
            return record.getLevel() == Level.SEVERE;
         }
      };
      addFileHandler("severe", severeFilter, formatter, diagnosticOutputDirectory, rootLogger);

      Filter warningFilter = new Filter()
      {
         @Override
         public boolean isLoggable(LogRecord record)
         {
            return record.getLevel() == Level.WARNING;
         }
      };
      addFileHandler("warning", warningFilter, formatter, diagnosticOutputDirectory, rootLogger);

      Filter noFilter = new Filter()
      {
         @Override
         public boolean isLoggable(LogRecord record)
         {
            return true;
         }
      };
      addFileHandler("all", noFilter, formatter, diagnosticOutputDirectory, rootLogger);

      Filter processedJointPositionDelayFiter = new Filter()
      {
         
         @Override
         public boolean isLoggable(LogRecord record)
         {
            return record instanceof ProcessedJointPositionDelayLogRecord;
         }
      };
      addFileHandler("processedJointPositionDelay", processedJointPositionDelayFiter, formatter, diagnosticOutputDirectory, rootLogger);

      Filter processedJointVelocityDelayFiter = new Filter()
      {
         
         @Override
         public boolean isLoggable(LogRecord record)
         {
            return record instanceof ProcessedJointVelocityDelayLogRecord;
         }
      };
      addFileHandler("processedJointVelocityDelay", processedJointVelocityDelayFiter, formatter, diagnosticOutputDirectory, rootLogger);

      Filter jointForceTrackingDelayFiter = new Filter()
      {
         
         @Override
         public boolean isLoggable(LogRecord record)
         {
            return record instanceof JointForceTrackingDelayLogRecord;
         }
      };
      addFileHandler("jointForceTrackingDelay", jointForceTrackingDelayFiter, formatter, diagnosticOutputDirectory, rootLogger);
   }

   private static void addFileHandler(String fileName, Filter filter, Formatter formatter, Path diagnosticOutputDirectory, Logger rootLogger)
   {
      try
      {
         String fullFileName = Paths.get(diagnosticOutputDirectory.toString(), fileName + ".log").toString();
         Handler fileHandler = new FileHandler(fullFileName);
         fileHandler.setFormatter(formatter);
         fileHandler.setFilter(filter);
         rootLogger.addHandler(fileHandler);
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
