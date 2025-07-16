package us.ihmc.simulationConstructionSetTools.tools;

import us.ihmc.log.LogTools;
import us.ihmc.simulationConstructionSetTools.util.gui.GUIMessageFrame;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class CITools
{
   private static final ExecutorService THREAD_POOL = Executors.newCachedThreadPool();

   private static boolean WRITE_LOG_FILE_ON_SUCCESS = false;

   public static boolean isNightlyBuild()
   {
      String buildType = System.getProperty("build.type");

      return ((buildType != null) && (buildType.equals("nightly")));
   }

   public static boolean isEveryCommitBuild()
   {
      String buildType = System.getProperty("build.type");

      return ((buildType != null) && (buildType.equals("everyCommit")));
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

      for (int i = 0; i < depth; i++)
      {
         if (elements.length > i)
         {
            StackTraceElement stackTraceElement = elements[i];
            if (stackTraceElement != null)
            {
               String methodName = stackTraceElement.getMethodName();
               String className = stackTraceElement.getClassName();
               className = className.substring(className.lastIndexOf('.') + 1);
               String classAndMethodName = className + "." + methodName;
               LogTools.info(classAndMethodName);
            }
         }
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

   public static void reportErrorMessage(String errorMessage, boolean showGUI)
   {
      LogTools.error(errorMessage);

      if (showGUI)
      {
         createGUIMessageFrame();
         guiMessageFrame.appendErrorMessage(errorMessage);
      }
   }

   public static void reportOutMessage(String outMessage, boolean showGUI)
   {
      LogTools.info(outMessage);

      if (showGUI)
      {
         createGUIMessageFrame();
         guiMessageFrame.appendOutMessage(outMessage);
      }
   }

   public static void reportParameterMessage(String parameterMessage, boolean showGUI)
   {
      LogTools.info(parameterMessage);

      if (showGUI)
      {
         createGUIMessageFrame();
         guiMessageFrame.appendParameterMessage(parameterMessage);
      }
   }

   public static void reportTestStartedMessage(boolean showGUI)
   {
      int usedMemoryInMB = garbageCollectAndGetUsedMemoryInMB();
      String message = CITools.getClassAndMethodName(1) + " started. Used Memory = " + usedMemoryInMB + " MB.";
      LogTools.info("Test started. {}", message);

      if (showGUI)
      {
         createGUIMessageFrame();
         guiMessageFrame.appendMessageToPanel(junitTestCasesIndex, message);
      }
   }

   public static void reportTestFinishedMessage(boolean showGUI)
   {
      int usedMemoryInMB = garbageCollectAndGetUsedMemoryInMB();
      String message = CITools.getClassAndMethodName(1) + " finished. Used Memory = " + usedMemoryInMB + " MB.";
      LogTools.info("Test finished. {}", message);

      if (showGUI)
      {
         createGUIMessageFrame();
         guiMessageFrame.appendMessageToPanel(junitTestCasesIndex, message);
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
         case NADIA:
            return "Nadia";
         case BONO:
            return "Bono";
         case SPOKED_RUNNER:
            return "SpokedRunner";
         case V2EXOSKELETON:
            return "V2Exoskeleton";
         case V3EXOSKELETON:
            return "V3Exoskeleton";
         case ALEXANDER:
            return "Alexander";
         default:
            return "";
      }
   }

   public static enum SimpleRobotNameKeys
   {
      M2V2, R2, VALKYRIE, ATLAS, NADIA, BONO, SPOKED_RUNNER, V2EXOSKELETON, V3EXOSKELETON, ALEXANDER
   }
}
