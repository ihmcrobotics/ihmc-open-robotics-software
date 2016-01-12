package us.ihmc.valkyrieRosControl.automatedDiagnosticControl;

import static us.ihmc.valkyrieRosControl.ValkyrieRosControlController.forceTorqueSensorModelNames;
import static us.ihmc.valkyrieRosControl.ValkyrieRosControlController.readForceTorqueSensors;
import static us.ihmc.valkyrieRosControl.ValkyrieRosControlController.readIMUs;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.HashMap;
import java.util.logging.FileHandler;
import java.util.logging.Filter;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

import org.apache.commons.io.output.TeeOutputStream;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.rosControl.JointHandle;
import us.ihmc.rosControl.valkyrie.ForceTorqueSensorHandle;
import us.ihmc.rosControl.valkyrie.IHMCValkyrieControlJavaBridge;
import us.ihmc.rosControl.valkyrie.IMUHandle;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.tools.SettableTimestampProvider;
import us.ihmc.util.PeriodicRealtimeThreadScheduler;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;
import us.ihmc.valkyrieRosControl.ValkyriePriorityParameters;

public class ValkyrieRosControlDiagnostic extends IHMCValkyrieControlJavaBridge
{
   private static final String[] controlledJoints = { "leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll",
         "rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll", "torsoYaw", "torsoPitch", "torsoRoll",
         "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw",
         "rightElbowPitch" };

   private final SettableTimestampProvider timestampProvider = new SettableTimestampProvider();

   public ValkyrieRosControlDiagnostic()
   {
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      Calendar calendar = Calendar.getInstance();
      String timestamp = dateFormat.format(calendar.getTime());

      Path diagnosticOutputDirectory = Paths.get(System.getProperty("user.home"), ".ihmc", "Diagnostic",
            timestamp + "_" + getClass().getSimpleName() + "_Outputs");

      setupSystemOut(diagnosticOutputDirectory);
      setupLogFiles(diagnosticOutputDirectory);
   }

   private void setupLogFiles(Path diagnosticOutputDirectory)
   {
      try
      {
         String severeFileName = Paths.get(diagnosticOutputDirectory.toString(), "severe.log").toString();
         Handler severeFileHandler = new FileHandler(severeFileName);
         Filter severeFilter = new Filter()
         {
            @Override
            public boolean isLoggable(LogRecord record)
            {
               return record.getLevel() == Level.SEVERE;
            }
         };
         severeFileHandler.setFilter(severeFilter);
         Logger.getGlobal().addHandler(severeFileHandler);
      }
      catch (SecurityException | IOException e)
      {
         e.printStackTrace();
      }

      try
      {
         String warningFileName = Paths.get(diagnosticOutputDirectory.toString(), "warning.log").toString();
         Handler warningFileHandler = new FileHandler(warningFileName);
         Filter warningFilter = new Filter()
         {
            @Override
            public boolean isLoggable(LogRecord record)
            {
               return record.getLevel() == Level.WARNING;
            }
         };
         warningFileHandler.setFilter(warningFilter);
         Logger.getGlobal().addHandler(warningFileHandler);
      }
      catch (SecurityException | IOException e)
      {
         e.printStackTrace();
      }

      try
      {
         String allFileName = Paths.get(diagnosticOutputDirectory.toString(), "all.log").toString();
         Handler allFileHandler = new FileHandler(allFileName);
         Logger.getGlobal().addHandler(allFileHandler);
      }
      catch (SecurityException | IOException e)
      {
         e.printStackTrace();
      }
   }

   private void setupSystemOut(Path diagnosticOutputDirectory)
   {
      Path consoleOutputPath = Paths.get(diagnosticOutputDirectory.toString(), "consoleOutput.log");

      try
      {
         Files.createDirectories(consoleOutputPath);
         // we will want to print in standard "System.out" and in "consoleOutput.log"
         TeeOutputStream newOut = new TeeOutputStream(System.out, new FileOutputStream(consoleOutputPath.toFile()));
         System.setOut(new PrintStream(newOut));
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   @Override
   protected void init()
   {
      long maxMemory = Runtime.getRuntime().maxMemory();

      System.out.println("Partying hard with max memory of: " + maxMemory);
      /*
       * Create joints
       */

      HashMap<String, JointHandle> jointHandles = new HashMap<>();
      for (String joint : controlledJoints)
      {
         jointHandles.put(joint, createJointHandle(joint));
      }

      HashMap<String, IMUHandle> imuHandles = new HashMap<>();
      for (String imu : readIMUs)
      {
         imuHandles.put(imu, createIMUHandle(imu));
      }

      HashMap<String, ForceTorqueSensorHandle> forceTorqueSensorHandles = new HashMap<>();
      for (int i = 0; i < readForceTorqueSensors.length; i++)
      {

         String forceTorqueSensor = readForceTorqueSensors[i];
         String modelName = forceTorqueSensorModelNames[i];
         forceTorqueSensorHandles.put(modelName, createForceTorqueSensorHandle(forceTorqueSensor));
      }

      /*
       * Create registries
       */

      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.REAL_ROBOT, true);
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();

      /*
       * Create network servers/clients
       */
      YoVariableServer yoVariableServer = new YoVariableServer(getClass(), new PeriodicRealtimeThreadScheduler(ValkyriePriorityParameters.LOGGER_PRIORITY),
            robotModel.getLogModelProvider(), robotModel.getLogSettings(ValkyrieConfigurationRoot.USE_CAMERAS_FOR_LOGGING), robotModel.getEstimatorDT());
   }

   @Override
   protected void doControl(long time, long duration)
   {
      timestampProvider.setTimestamp(time);
   }
}
