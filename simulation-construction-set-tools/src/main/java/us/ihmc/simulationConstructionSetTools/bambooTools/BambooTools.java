package us.ihmc.simulationConstructionSetTools.bambooTools;

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

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationConstructionSetTools.util.gui.GUIMessageFrame;
import us.ihmc.commons.FormattingTools;

public class BambooTools
{
   private final static String[] possibleRootDirectoriesForBambooDataAndVideos = new String[] { "C:/videos/", "D:/BambooDataAndVideos/",
         "../BambooDataAndVideos/", "~/bamboo-videos" };

   private final static String eraseableBambooDataAndVideosDirectoryLinux = "~/bamboo-videos";
   private final static String eraseableBambooDataAndVideosDirectoryWindows = "C:/videos/";

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

   /**
    * This method will generate videos of the current simulation and store it in the default path defined in this class. 
    * It will use the stack trace to find the class name and method name to generate the file name. The additionalStackDepthForRelevantCallingMethod allows you to specify 
    * how far up the stack trace the method will search to find the class and test name.
    * @param simplifiedRobotModelName the name of the robot, used in the file name of the exported video
    * @param scs the simulation used to create the video
    * @param additionalStackDepthForRelevantCallingMethod the index of the stack trace to pull the class and test name
    */
   public static void createVideoWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(String simplifiedRobotModelName, SimulationConstructionSet scs,
         int additionalStackDepthForRelevantCallingMethod)
   {
      //Get file name from stack trace
      String classAndMethodName = getClassAndMethodName(additionalStackDepthForRelevantCallingMethod + 1);
      String fileName = simplifiedRobotModelName + "_" + classAndMethodName;
      
      createVideoWithDateTimeAndStoreInDefaultDirectory(scs, fileName);
   }

   /**
    * This method will generate videos of the current simulation and store it in the default path defined in this class. 
    * It will append a date and time to the video name and add the video extension for you. Our video tools are pretty dependent 
    * on this convention so please double check the output is correct
    * 
    * To be consistent with the auto generated name, please use the following format:
    * RobotName_ClassName.TestMethodName
    * 
    * example:
    * 
    * Atlas_AtlasEndToEndFootTrajectoryMessageTest.testSingleWaypoint
    */
   public static void createVideoWithDateTimeAndStoreInDefaultDirectory(SimulationConstructionSet scs, String videoName)
   {
      PrintTools.info("Trying to create " +  videoName + " video!");
      
      try
      {
         String rootDirectoryToUse = determineEraseableBambooDataAndVideosRootDirectoryToUse();
         
         if (rootDirectoryToUse == null)
         {
            reportErrorMessage("Couldn't find a BambooDataAndVideos directory (for share drive)!", scs.getSimulationConstructionSetParameters().getShowWindows());
            return;
         }
         
         reportOutMessage("Automatically creating video and data and saving to " + rootDirectoryToUse, scs.getSimulationConstructionSetParameters().getShowWindows());

         createVideoWithDateTimeAndName(rootDirectoryToUse, scs, false, videoName);
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
   
   
   /**
    * Internal method that calls on scs to create video and data
    * @param rootDirectory - directory to store the video and data
    * @param scs - the sim to use to create the video
    * @param writeData - whether or not to create sim data
    * @param videoName - the name of the video (does not include date,time, and extension yet)
    * @return File handles to the directory, video file, and the data file, in that order. Access to the file handle does not mean the file was created successfully. 
    */
   private static File[] createVideoWithDateTimeAndName(String rootDirectory, SimulationConstructionSet scs, boolean writeData, String videoName)
   {
      String dateString = FormattingTools.getDateString();
      String directoryName = rootDirectory + dateString + "/";

      File directory = new File(directoryName);
      if (!directory.exists())
      {
         directory.mkdir();
      }

      String timeString = FormattingTools.getTimeString();
      String dateTimeString = dateString + "_" + timeString;
     
      String videoFilename = dateTimeString + "_" + videoName + ".mp4";

      PrintTools.debug(videoFilename);
      
      File videoFile = scs.createVideo(directoryName + videoFilename);

      String dataFilename = directoryName + dateTimeString + ".data.gz";

      File dataFile = new File(dataFilename);

      if(writeData)
      {
         try
         {
            scs.writeData(dataFile);
         }
         catch (Exception e)
         {
            System.err.println("Error in writing data file in BambooTools.createVideoAndDataWithDateTimeClassMethod()");
            e.printStackTrace();
         }
      }

      if (doVideoUpload())
      {
         throw new RuntimeException("Youtube API v2 got deleted. Re-implement using v3 API");
      }

      scs.gotoOutPointNow();

      return new File[] { directory, videoFile, dataFile };      
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
      if(rootDirectoryToTry != null)
      {
         if(!rootDirectoryToTry.endsWith("/") && !rootDirectoryToTry.endsWith("\\"))
         {
            rootDirectoryToTry += "/";
         }
      }
      
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

         Path temporaryDirectoryPath = PathTools.systemTemporaryDirectory();
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
      //Print out some helpful info, if the stack depth is wrong you need to look at the whole stack trace to find the source
      printStackTrace(5);
      
      int stackDepthForRelevantCallingMethod = stackDepthZeroIfCallingMethod + 2;

      StackTraceElement[] elements = Thread.currentThread().getStackTrace();
      String methodName = elements[stackDepthForRelevantCallingMethod].getMethodName();
      String className = elements[stackDepthForRelevantCallingMethod].getClassName();
      className = className.substring(className.lastIndexOf('.') + 1);

      String classAndMethodName = className + "." + methodName;

      return classAndMethodName;
   }
   
   private static void printStackTrace(int depth)
   {
      StackTraceElement[] elements = Thread.currentThread().getStackTrace();
      
      for(int i = 0; i < depth; i++)
      {
         if(elements.length > i)
         {
            StackTraceElement stackTraceElement = elements[i];
            if(stackTraceElement != null)
            {
               String methodName = stackTraceElement.getMethodName();
               String className = stackTraceElement.getClassName();
               className = className.substring(className.lastIndexOf('.') + 1);
               String classAndMethodName = className + "." + methodName;
               PrintTools.info(classAndMethodName);
            }
         }
      }
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

      String logFilename = rootDirectoryToUse + FormattingTools.getDateString() + "_" + FormattingTools.getTimeString() + "_" + filename;
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
      case V3EXOSKELETON:
         return "V3Exoskeleton";
      default:
         return "";
      }
   }

   public static enum SimpleRobotNameKeys
   {
      M2V2, R2, VALKYRIE, ATLAS, BONO, SPOKED_RUNNER, V2EXOSKELETON,V3EXOSKELETON
   }
}
