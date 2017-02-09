package us.ihmc.simulationconstructionset.bambooTools;

import java.io.File;
import java.io.FileFilter;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintStream;
import java.net.URL;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Comparator;

import org.apache.commons.lang3.SystemUtils;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.gui.GUIMessageFrame;
import us.ihmc.tools.io.files.FileTools;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.time.DateTools;

public class BambooTools
{
   private final static String[] possibleRootDirectoriesForBambooDataAndVideos = new String[] { "C:/BambooDataAndVideos/", "D:/BambooDataAndVideos/",
         "../BambooDataAndVideos/", "~/bamboo-videos" };

   private final static String eraseableBambooDataAndVideosDirectoryLinux = "~/bamboo-videos";
   private final static String eraseableBambooDataAndVideosDirectoryWindows = "X:/EraseableBambooDataAndVideos/";

   private static final String UPLOADED_VIDEOS_LOG = "uploaded-videos.log";

   private static boolean WRITE_LOG_FILE_ON_SUCCESS = false;

   public static boolean isNightlyBuild()
   {
      String buildType = System.getProperty("build.type");

      return ((buildType != null) && (buildType.equals("nightly")));
   }

   /**
    * If you set {@code upload.videos} to {@code true} you must also set the system properties
    * defined in {@link YouTubeCredentials}.
    */
   public static boolean doVideoUpload()
   {
      return Boolean.parseBoolean(System.getProperty("upload.videos"));
   }

   public static boolean isEveryCommitBuild()
   {
      String buildType = System.getProperty("build.type");

      return ((buildType != null) && (buildType.equals("everyCommit")));
   }

   public static boolean isIsolatedBuild()
   {
      String buildType = System.getProperty("build.type");

      return ((buildType != null) && (buildType.equals("isolated")));
   }

   public static void createVideoAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(String simplifiedRobotModelName, SimulationConstructionSet scs)
   {
      createVideoAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(simplifiedRobotModelName, scs, 1);
   }

   public static void createVideoAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(String simplifiedRobotModelName, SimulationConstructionSet scs,
         int additionalStackDepthForRelevantCallingMethod)
   {
      PrintTools.info("Trying to create video!");
      try
      {
         String rootDirectoryToUse = determineEraseableBambooDataAndVideosRootDirectoryToUse();

         if (rootDirectoryToUse == null)
         {
            reportErrorMessage("Couldn't find a BambooDataAndVideos directory (for share drive)!", scs.getSimulationConstructionSetParameters().getShowWindows());

            return;
         }

         reportOutMessage("Automatically creating video and data and saving to " + rootDirectoryToUse, scs.getSimulationConstructionSetParameters().getShowWindows());

         createVideoAndDataWithDateTimeClassMethod(rootDirectoryToUse, simplifiedRobotModelName, scs, additionalStackDepthForRelevantCallingMethod + 2);
      }
      catch (Throwable t)
      {
         reportErrorMessage("createVideoAndData failed with " + t.toString(), scs.getSimulationConstructionSetParameters().getShowWindows());
         t.printStackTrace();
         System.err.flush();
         if (t instanceof Error)
         {
            throw (Error) t;
         }
         else
         {
            throw (RuntimeException) t;
         }
      }
   }

   private static String determineBambooDataAndVideosRootDirectoryToUse()
   {
      String rootDirectoryToUse = null;

      for (String rootDirectoryToTry : possibleRootDirectoriesForBambooDataAndVideos)
      {
         File rootFile = new File(rootDirectoryToTry);
         if (rootFile.exists())
         {
            rootDirectoryToUse = rootDirectoryToTry;

            break;
         }
      }

      return rootDirectoryToUse;
   }

   private static String determineEraseableBambooDataAndVideosRootDirectoryToUse()
   {
      String rootDirectoryToTry = System.getProperty("create.videos.dir");
      if (rootDirectoryToTry == null)
      {
         if (SystemUtils.IS_OS_WINDOWS)
         {
            rootDirectoryToTry = eraseableBambooDataAndVideosDirectoryWindows;
         }
         else
         {
            rootDirectoryToTry = eraseableBambooDataAndVideosDirectoryLinux;
         }
      }

      if (new File(rootDirectoryToTry).exists())
      {
         return rootDirectoryToTry;
      }
      else if (doVideoUpload())
      {
         // if we're going to upload the videos, we can just use the tmp dir if
         // the other specified directories don't exist
         System.out.println("Saving videos to tmp dir before uploading..");

         Path temporaryDirectoryPath = FileTools.getTemporaryDirectoryPath();
         File videoDir = temporaryDirectoryPath.resolve("atlas-videos").toFile();
         if (videoDir.exists() || videoDir.mkdirs())
         {
            System.out.println("Using " + videoDir.getAbsolutePath());
            return videoDir.getAbsolutePath();
         }
         System.err.println("Couldn't create directory: " + videoDir.getAbsolutePath());
      }

      return determineBambooDataAndVideosRootDirectoryToUse();
   }

   public static Path getSVNDirectoryWithMostRecentBambooDataAndVideos()
   {
      String rootDirectory = determineBambooDataAndVideosRootDirectoryToUse();

      return getDirectoryWithMostRecentBambooDataAndVideos(Paths.get(rootDirectory));
   }

   public static Path getEraseableDirectoryWithMostRecentBambooDataAndVideos()
   {
      String rootDirectory = determineEraseableBambooDataAndVideosRootDirectoryToUse();

      PrintTools.info(rootDirectory);

      return getDirectoryWithMostRecentBambooDataAndVideos(Paths.get(rootDirectory));
   }

   public static Path getDirectoryWithMostRecentBambooDataAndVideos(Path rootDirectory)
   {
      if (rootDirectory == null)
         return null;

      File file = rootDirectory.toFile();

      FileFilter fileFilter = new FileFilter()
      {
         @Override
         public boolean accept(File file)
         {
            if (!file.isDirectory())
               return false;
            String fileName = file.getName();

            String regex = "\\d\\d\\d\\d\\d\\d\\d\\d";
            if (fileName.matches(regex))
               return true;

            return false;
         }

      };

      File[] timeStampedDirectories = file.listFiles(fileFilter);
      if (timeStampedDirectories == null)
         return null;
      if (timeStampedDirectories.length == 0)
         return null;

      Comparator<File> fileAlphabeticalComparator = createFileAlphabeticalComparator();
      Arrays.sort(timeStampedDirectories, fileAlphabeticalComparator);

      return timeStampedDirectories[0].toPath();
   }

   public static Comparator<File> createFileAlphabeticalComparator()
   {
      Comparator<File> fileAlphabeticalComparator = new Comparator<File>()
      {
         @Override
         public int compare(File file1, File file2)
         {
            String name1 = file1.getName();
            String name2 = file2.getName();

            return name2.compareTo(name1);
         }

      };

      return fileAlphabeticalComparator;
   }

   private static File[] createVideoAndDataWithDateTimeClassMethod(String rootDirectory, String simplifiedRobotModelName, SimulationConstructionSet scs,
         int stackDepthForRelevantCallingMethod)
   {
      String dateString = DateTools.getDateString();
      String directoryName = rootDirectory + dateString + "/";

      File directory = new File(directoryName);
      if (!directory.exists())
      {
         directory.mkdir();
      }

      String classAndMethodName = getClassAndMethodName(stackDepthForRelevantCallingMethod);

      String timeString = DateTools.getTimeString();
      String filenameStart = dateString + "_" + timeString;
      if (!simplifiedRobotModelName.equals(""))
      {
         filenameStart += "_" + simplifiedRobotModelName;
      }
      filenameStart += "_" + classAndMethodName;
      String videoFilename = filenameStart + ".mp4";

      //    String videoFilename = filenameStart + ".mov";
      File videoFile = scs.createVideo(directoryName + videoFilename);

      String dataFilename = directoryName + filenameStart + ".data.gz";

      File dataFile = new File(dataFilename);

      try
      {
         scs.writeData(dataFile);
      }
      catch (Exception e)
      {
         System.err.println("Error in writing data file in BambooTools.createVideoAndDataWithDateTimeClassMethod()");
         e.printStackTrace();
      }

      if (doVideoUpload())
      {
         throw new RuntimeException("Youtube API v2 got deleted. Re-implement using v3 API");
      }

      scs.gotoOutPointNow();

      return new File[] { directory, videoFile, dataFile };
   }

   private static void writeVideoUrlToVideoLog(String fileName, String videoUrl)
   {
      String artifactsOut = System.getProperty("artifacts.out");
      if (artifactsOut == null)
      {
         System.err.println("No artifacts.out system property set, not logging video url.");
         return;
      }

      // create artifacts dir if it doesn't already exist
      new File(artifactsOut).mkdirs();

      File file = new File(artifactsOut, UPLOADED_VIDEOS_LOG);
      try
      {
         OutputStreamWriter out = new OutputStreamWriter(new FileOutputStream(file, true));
         out.write(videoUrl + " (" + fileName + ")\r\n");
         out.flush();
         out.close();
      }
      catch (IOException e)
      {
         System.err.println("Failed to write video url to " + file.getAbsolutePath() + "\n" + e.toString());
      }
   }

   public static String getClassAndMethodName()
   {
      return getClassAndMethodName(1);
   }

   public static String getClassAndMethodName(int stackDepthZeroIfCallingMethod)
   {
      int stackDepthForRelevantCallingMethod = stackDepthZeroIfCallingMethod + 2;

      StackTraceElement[] elements = Thread.currentThread().getStackTrace();
      String methodName = elements[stackDepthForRelevantCallingMethod].getMethodName();
      String className = elements[stackDepthForRelevantCallingMethod].getClassName();
      className = className.substring(className.lastIndexOf('.') + 1);

      String classAndMethodName = className + "." + methodName;

      return classAndMethodName;
   }

   private static void writeSuccessLogFile(String successString, String logFilename)
   {
      if (!WRITE_LOG_FILE_ON_SUCCESS)
         return;

      System.out.println("Writing " + successString + " to log file " + logFilename);
      File logFile = new File(logFilename);

      try
      {
         FileOutputStream logStream = new FileOutputStream(logFile);
         PrintStream printStream = new PrintStream(logStream);

         printStream.println(successString);
         printStream.close();

         System.out.println("Done writing " + successString + " to log file " + logFilename);

      }
      catch (FileNotFoundException e1)
      {
         System.out.println("FileNotFoundException! File = " + logFile);
      }
   }

   private static void writeErrorLogFile(Exception exception, String logFilename)
   {
      System.out.println("Writing error log to log file " + logFilename + ". Exception was " + exception);

      File logFile = new File(logFilename);

      try
      {
         FileOutputStream logStream = new FileOutputStream(logFile);
         PrintStream printStream = new PrintStream(logStream);

         exception.printStackTrace(printStream);
         printStream.close();
      }
      catch (FileNotFoundException e1)
      {
      }
   }

   public static int garbageCollectAndGetUsedMemoryInMB()
   {
      Runtime runtime = Runtime.getRuntime();

      System.gc();
      sleep(100);
      System.gc();

      long freeMemory = runtime.freeMemory();
      long totalMemory = runtime.totalMemory();
      long usedMemory = totalMemory - freeMemory;

      int usedMemoryMB = (int) (usedMemory / 1000000);

      return usedMemoryMB;
   }

   private static void sleep(long sleepMillis)
   {
      try
      {
         Thread.sleep(sleepMillis);
      }
      catch (InterruptedException e)
      {
      }
   }

   private static GUIMessageFrame guiMessageFrame;
   private static int junitTestCasesIndex;

   public static void logMessagesToFile(File file)
   {
      if (guiMessageFrame == null)
      {
         PrintTools.error("GUI message frame null. Returning");
         return;
      }

      guiMessageFrame.save(file);
   }

   public static void logMessagesAndShareOnSharedDriveIfAvailable(String filename)
   {
      String rootDirectoryToUse = determineEraseableBambooDataAndVideosRootDirectoryToUse();

      String logFilename = rootDirectoryToUse + DateTools.getDateString() + "_" + DateTools.getTimeString() + "_" + filename;
      File file = new File(logFilename);
      logMessagesToFile(file);
   }

   public static void reportErrorMessage(String errorMessage, boolean showGUI)
   {
      System.err.println(errorMessage);

      if (showGUI)
      {
         createGUIMessageFrame();
         guiMessageFrame.appendErrorMessage(errorMessage);
      }
   }

   public static void reportOutMessage(String outMessage, boolean showGUI)
   {
      System.out.println(outMessage);

      if (showGUI)
      {
         createGUIMessageFrame();
         guiMessageFrame.appendOutMessage(outMessage);
      }
   }

   public static void reportParameterMessage(String parameterMessage, boolean showGUI)
   {
      System.out.println(parameterMessage);

      if (showGUI)
      {
         createGUIMessageFrame();
         guiMessageFrame.appendParameterMessage(parameterMessage);
      }
   }

   public static void reportTestStartedMessage(boolean showGUI)
   {
      int usedMemoryInMB = garbageCollectAndGetUsedMemoryInMB();

      if (showGUI)
      {
         createGUIMessageFrame();
         guiMessageFrame.appendMessageToPanel(junitTestCasesIndex, BambooTools.getClassAndMethodName(1) + " started. Used Memory = " + usedMemoryInMB + " MB.");
      }
   }

   public static void reportTestFinishedMessage(boolean showGUI)
   {
      int usedMemoryInMB = garbageCollectAndGetUsedMemoryInMB();

      if (showGUI)
      {
         createGUIMessageFrame();
         guiMessageFrame
               .appendMessageToPanel(junitTestCasesIndex, BambooTools.getClassAndMethodName(1) + " finished. Used Memory = " + usedMemoryInMB + " MB.");
      }
   }

   private static void createGUIMessageFrame()
   {
      if (guiMessageFrame == null)
      {
         guiMessageFrame = GUIMessageFrame.getInstance();
         junitTestCasesIndex = guiMessageFrame.createGUIMessagePanel("JUnit Test Cases");
      }
   }

   public static void sleepForever()
   {
      while (true)
      {
         sleep(1.00);
      }

   }

   public static void sleep(double secondsToSleep)
   {
      try
      {
         Thread.sleep((long) (secondsToSleep * 1000));
      }
      catch (InterruptedException e)
      {
      }
   }

   public static String getFullFilenameUsingClassRelativeURL(Class<?> class1, String relativeFileName)
   {
      URL resource = class1.getResource(relativeFileName);
      if (resource == null)
         throw new RuntimeException("resource " + relativeFileName + " == null");
      String fileName = resource.getFile();

      return fileName;
   }

   public static String getSimpleRobotNameFor(SimpleRobotNameKeys key)
   {
      switch (key)
      {
      case M2V2:
         return "M2V2";
      case R2:
         return "R2";
      case VALKYRIE:
         return "Valkyrie";
      case ATLAS:
         return "Atlas";
      case BONO:
         return "Bono";
      case SPOKED_RUNNER:
         return "SpokedRunner";
      case V2EXOSKELETON:
         return "V2Exoskeleton";
      default:
         return "";
      }
   }

   public static enum SimpleRobotNameKeys
   {
      M2V2, R2, VALKYRIE, ATLAS, BONO, SPOKED_RUNNER, V2EXOSKELETON
   }

}
