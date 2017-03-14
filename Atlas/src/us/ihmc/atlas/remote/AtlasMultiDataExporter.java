package us.ihmc.atlas.remote;

import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.util.ArrayList;

import javax.swing.Action;
import javax.swing.ActionMap;
import javax.swing.JFileChooser;
import javax.swing.JOptionPane;
import javax.swing.JTable;
import javax.swing.filechooser.FileNameExtensionFilter;

import org.apache.batik.dom.util.HashTable;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.robotDataLogger.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.robotDataLogger.logger.YoVariableLoggerListener;
import us.ihmc.robotDataVisualizer.logger.MultiVideoDataPlayer;
import us.ihmc.robotDataVisualizer.logger.YoVariableLogPlaybackRobot;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.DataBufferEntry;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.SimulationDoneListener;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.tools.gui.SwingUtils;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasMultiDataExporter implements SimulationDoneListener
{
   private static final long CLOSING_SLEEP_TIME = 2000;

   YoVariableLogPlaybackRobot robot;
   SimulationConstructionSet scs;
   YoVariableHandshakeParser parser;
   MultiVideoDataPlayer players = null;
   SimulateAndExport simulateExport;
   Thread simulation;
   boolean exportSCSVideo;
   boolean showCameraVideo;

   public AtlasMultiDataExporter(DRCRobotModel robotModel, int bufferSize, boolean showGUIAndSaveSCSVideo, boolean showCameraVideo, File logFile)
         throws IOException
   {
      initialize(robotModel, bufferSize, showGUIAndSaveSCSVideo, logFile);
      simulation = new Thread(scs);
      exportSCSVideo = showGUIAndSaveSCSVideo;
      this.showCameraVideo = showCameraVideo;
   }

   public static void main(String[] args) throws IOException
   {
      final AtlasRobotVersion ATLAS_ROBOT_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
      int numberOfEntries = 0;
      DRCRobotModel model = new AtlasRobotModel(ATLAS_ROBOT_VERSION, DRCRobotModel.RobotTarget.SCS, false);
      DRCRobotJointMap jointMap = model.getJointMap();
      ArmJointName[] joints = jointMap.getArmJointNames();
      FullHumanoidRobotModel robotModel = model.createFullRobotModel();
      boolean showGUIAndSaveSCSVideo = false;
      boolean showCameraVideo = false;

      String[] cameraName;
      double[] simulationCameraHeight;
      double[] simulationCameraRadius;
      double[] simulationCameraHour;
      String[] outputFileName;
      File[] inFolder;
      File[] outFolder;
      int[] bufferSize;
      int[] startIndex;
      double[] secondOfSimulation;
      String[] timeVariable;

      // variables to export (starts from 1: vars[0] is reserved for timeVariable)
      String[] vars = new String[49];
      int i = 0;
      int jj = 0;

      for(RobotSide side : RobotSide.values())
      {
         jj = 0;
         for (ArmJointName joint: joints)
         {
            String jointName = robotModel.getArmJoint(side, joint).getName();

            vars[i * joints.length * 4 + jj + 1] = "ll_in_" + jointName + "_qd_bef";
            jj++;
            vars[i * joints.length * 4 + jj + 1] = "ll_in_" + jointName + "_qd_aft";
            jj++;
            vars[i * joints.length * 4 + jj + 1] = "ll_in_" + jointName + "_f";
            jj++;
            vars[i * joints.length * 4 + jj + 1] = "ll_in_" + jointName + "_q_bef";
            jj++;
         }
         i++;
      }

      // select the file with export parameters
      File inputParameters = selectInputFile();
      if (inputParameters != null)
      {
         HashTable table = readInputFile(inputParameters);

         numberOfEntries = (int) table.get("numberOfEntries");
         cameraName = (String[]) table.get("cameraName");
         simulationCameraHeight = (double[]) table.get("simulationCameraHeight");
         simulationCameraRadius = (double[]) table.get("simulationCameraRadius");
         simulationCameraHour = (double[]) table.get("simulationCameraHour");
         outputFileName = (String[]) table.get("outputFileName");
         inFolder = (File[]) table.get("inFolder");
         outFolder = (File[]) table.get("outFolder");
         bufferSize = (int[]) table.get("bufferSize");
         startIndex = (int[]) table.get("startIndex");
         secondOfSimulation = (double[]) table.get("secondOfSimulation");
         timeVariable = (String[]) table.get("timeVariable");
      }
      else
      {
         System.out.println("Input parameters are not valid");
         return;
      }

      for (int j = 0; j < numberOfEntries; j++)
      {
         vars[0] = timeVariable[j];
         double[] cameraParameters = { simulationCameraHour[j], simulationCameraRadius[j], simulationCameraHeight[j] };
         final String variableGroupName = "selectedVariables";

         AtlasMultiDataExporter exportData = new AtlasMultiDataExporter(model, bufferSize[j], showGUIAndSaveSCSVideo, showCameraVideo, inFolder[j]);

         SimulateAndExport simulateExport = exportData.new SimulateAndExport(exportData, startIndex[j], secondOfSimulation[j], cameraName[j],
               outputFileName[j], outFolder[j], variableGroupName, vars, timeVariable[j], cameraParameters);
         exportData.setSimulateExport(simulateExport);
         exportData.simulateExport.startSimulation();

         while (!exportData.simulateExport.exportFinish)
         {
            System.out.println("Export not finished");
            try
            {
               Thread.sleep(60000);
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }
         }

         System.out.println("Simulation finished");
         simulateExport.closeSCS();
         ThreadTools.sleep(CLOSING_SLEEP_TIME);
      }
   }

   private static File selectInputFile()
   {
      final String defaultLogReadingDirectory = System.getProperty("user.home");
      final JFileChooser fileChooser = new JFileChooser(new File(defaultLogReadingDirectory));
      File file = null;
      fileChooser.setFileFilter(new FileNameExtensionFilter("TEXT FILES", "txt", "text"));
      fileChooser.setDialogTitle("Select the save parameter file");
      int returnValue = fileChooser.showOpenDialog(null);
      if (returnValue == JFileChooser.APPROVE_OPTION)
      {
         file = fileChooser.getSelectedFile();
      }
      else
      {
         System.err.println("No file selected, closing.");
      }
      return file;
   }

   private static HashTable readInputFile(File input) throws IOException
   {
      HashTable exportParameters = new HashTable();
      String[] cameraName;
      double[] simulationCameraHeight;
      double[] simulationCameraRadius;
      double[] simulationCameraHour;
      String[] outputFileName;
      File[] inFolder;
      File[] outFolder;
      int[] bufferSize;
      int[] startIndex;
      double[] secondOfSimulation;
      String[] timeVariable;
      int numberOfEntries;

      String count;
      try
      {
         BufferedReader d = new BufferedReader(new InputStreamReader(new FileInputStream(input)));

         String header = d.readLine();
         if (header == null)
         {
            System.out.println("Can not read input file!");
            return null;
         }

         numberOfEntries = Integer.parseInt(header.split(",")[11]);

         cameraName = new String[numberOfEntries];
         simulationCameraHeight = new double[numberOfEntries];
         simulationCameraRadius = new double[numberOfEntries];
         simulationCameraHour = new double[numberOfEntries];
         outputFileName = new String[numberOfEntries];
         inFolder = new File[numberOfEntries];
         outFolder = new File[numberOfEntries];
         bufferSize = new int[numberOfEntries];
         startIndex = new int[numberOfEntries];
         secondOfSimulation = new double[numberOfEntries];
         timeVariable = new String[numberOfEntries];

         for (int i = 0; i < numberOfEntries; i++)
         {
            count = d.readLine();
            if (count == null) { break; }

            String[] splittedLine = count.split(",");
            cameraName[i] = splittedLine[0];
            simulationCameraHeight[i] = Double.parseDouble(splittedLine[1]);
            simulationCameraRadius[i] = Double.parseDouble(splittedLine[2]);
            simulationCameraHour[i] = Double.parseDouble(splittedLine[3]);
            outputFileName[i] = splittedLine[4];
            inFolder[i] = new File(splittedLine[5]);
            outFolder[i] = new File(splittedLine[6]);
            bufferSize[i] = Integer.parseInt(splittedLine[7]);
            startIndex[i] = Integer.parseInt(splittedLine[8]);
            secondOfSimulation[i] = Double.parseDouble(splittedLine[9]);
            timeVariable[i] = splittedLine[10];
         }

         d.close();
      }
      catch (IOException e)
      {
         System.out.println("Can not read input file: " + e.getMessage());
         return null;
      }

      exportParameters.put("cameraName", cameraName);
      exportParameters.put("simulationCameraHeight", simulationCameraHeight);
      exportParameters.put("simulationCameraRadius", simulationCameraRadius);
      exportParameters.put("simulationCameraHour", simulationCameraHour);
      exportParameters.put("outputFileName", outputFileName);
      exportParameters.put("inFolder", inFolder);
      exportParameters.put("outFolder", outFolder);
      exportParameters.put("bufferSize", bufferSize);
      exportParameters.put("startIndex", startIndex);
      exportParameters.put("secondOfSimulation", secondOfSimulation);
      exportParameters.put("timeVariable", timeVariable);
      exportParameters.put("numberOfEntries", numberOfEntries);

      return exportParameters;
   }

   private static File selectDirectories()
   {
      final String defaultLogReadingDirectory = System.getProperty("user.home");
      final JFileChooser fileChooser = new JFileChooser(new File(defaultLogReadingDirectory));
      File file = null;
      sortByDateHack(fileChooser);
      fileChooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
      int returnValue = fileChooser.showOpenDialog(null);
      if (returnValue == JFileChooser.APPROVE_OPTION)
      {
         file = fileChooser.getSelectedFile();
      }
      else
      {
         System.err.println("No file selected, closing.");
      }
      return file;
   }

   private void initialize(DRCRobotModel robotModel, int bufferSize, boolean showGUI, File logFile) throws IOException
   {
      if (logFile == null)
      {
         logFile = selectDirectories();
      }

      if (logFile != null)
      {
         System.out.println("loading log from folder:" + logFile);

         SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
         parameters.setCreateGUI(showGUI);
         parameters.setDataBufferSize(bufferSize);
         scs = new SimulationConstructionSet(parameters);
         readLogFile(logFile, robotModel);
      }
      else
      {
         scs = null;
      }
   }

   private static void sortByDateHack(final JFileChooser fileChooser)
   {
      ActionMap actionMap = fileChooser.getActionMap();
      Action details = actionMap.get("viewTypeDetails");
      if (details != null)
      {
         details.actionPerformed(null);

         JTable table = SwingUtils.getDescendantsOfType(JTable.class, fileChooser).get(0);
         table.getRowSorter().toggleSortOrder(2);
         table.getRowSorter().toggleSortOrder(2);
      }
      else
      {
         System.err.println("sort by datetime doesn't work known on OSX ... bail out");
      }
   }

   private void readLogFile(File selectedFile, DRCRobotModel robotModel) throws IOException
   {
      LogPropertiesReader logProperties = new LogPropertiesReader(new File(selectedFile, YoVariableLoggerListener.propertyFile));
      File handshake = new File(selectedFile, logProperties.getHandshakeFile());
      if (!handshake.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getHandshakeFile());
      }

      DataInputStream handshakeStream = new DataInputStream(new FileInputStream(handshake));
      byte[] handshakeData = new byte[(int) handshake.length()];
      handshakeStream.readFully(handshakeData);
      handshakeStream.close();

      parser = new YoVariableHandshakeParser("logged");
      parser.parseFrom(handshakeData);

//      boolean useCollisionMeshes = false;
//      boolean enableTorqueVelocityLimits = true;
//      boolean enableJointDamping = true;
//
//      GeneralizedSDFRobotModel generalizedSDFRobotModel = robotModel.getGeneralizedRobotModel();
//      RobotDescriptionFromSDFLoader loader = new RobotDescriptionFromSDFLoader();
//      RobotDescription robotDescription = loader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, null, useCollisionMeshes, enableTorqueVelocityLimits, enableJointDamping);

      RobotDescription robotDescription = robotModel.getRobotDescription();

      robot = new YoVariableLogPlaybackRobot(selectedFile, robotDescription, parser.getJointStates(),
            parser.getYoVariablesList(), logProperties, scs);

      double dt = parser.getDt();
      System.out.println(getClass().getSimpleName() + ": dt set to " + dt);
      scs.setDT(dt, 1);
      scs.setPlaybackDesiredFrameRate(0.04);

      YoGraphicsListRegistry yoGraphicsListRegistry = parser.getYoGraphicsListRegistry();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.getRootRegistry().addChild(parser.getRootRegistry());
      scs.setGroundVisible(false);

      try
      {
         players = new MultiVideoDataPlayer(selectedFile, logProperties, robot.getTimestamp());
         if (!showCameraVideo)
         {
            players.setActivePlayer(null);
         }
         scs.attachPlaybackListener(players);
         scs.attachSimulationRewoundListener(players);
      }
      catch (Exception e)
      {
         System.err.println("Couldn't load video file!");
         e.printStackTrace();
      }

      scs.hideAllYoGraphics();
      scs.setYoGraphicsListVisible("DesiredExternalWrench", true);
   }

   public void setSimulateExport(SimulateAndExport simulateExport)
   {
      this.simulateExport = simulateExport;
   }

   @Override
   public void simulationDone()
   {
      System.out.println("Simulation done!");
      this.simulateExport.exportDataAndVideo();
   }

   @Override
   public void simulationDoneWithException(Throwable throwable)
   {
      simulationDone();
   }

   private class SimulateAndExport
   {
      AtlasMultiDataExporter exportData;
      int inPointIndex;
      double secondsOfSimulation;
      String variableGroupName;
      String[] vars;
      String cameraName;
      String fileName;
      String savePath;
      String timeVariable;
      double[] cameraParameters;
      boolean exportFinish;

      public SimulateAndExport(AtlasMultiDataExporter exportData, int inPointIndex, double secondsOfSimulation, String cameraName, String fileName,
            File outFolder, String variableGroupName, String[] vars, String timeVariable, double[] cameraParameters)
      {
         if (outFolder == null)
         {
            outFolder = selectDirectories();
         }
         this.exportFinish = false;
         this.exportData = exportData;
         this.inPointIndex = inPointIndex;
         this.secondsOfSimulation = secondsOfSimulation;
         this.variableGroupName = variableGroupName;
         this.vars = vars;
         this.cameraName = cameraName;
         this.fileName = fileName;
         this.savePath = outFolder.getPath();
         this.timeVariable = timeVariable;
         this.cameraParameters = cameraParameters;
      }

      private void startSimulation()
      {
         exportData.simulation.start();
         exportData.scs.setupVarGroup(variableGroupName, vars);

         // set the index of the slider and replay the logged data from the simulation
         gotoPoint(inPointIndex, exportData);
         exportData.scs.setInPoint();
         setCamera(exportData, cameraParameters);
         exportData.scs.simulate(secondsOfSimulation);
         exportData.scs.addSimulateDoneListener(exportData);
      }

      private void closeSCS()
      {
         ThreadTools.sleep(CLOSING_SLEEP_TIME);
         scs.closeAndDispose();
         scs = null;
      }

      private void exportDataAndVideo()
      {
         exportData.scs.setOutPoint();
         exportData.scs.cropBuffer();

         if (exportData.exportSCSVideo)
            exportData.scs.createVideo(savePath + File.separator + fileName + "SCS");

         switchVideoUpdate(exportData, cameraName);
         exportVideo(exportData, savePath, fileName + cameraName);

         File dataFile = new File(savePath + File.separator + fileName + ".csv");
         writeSpreadsheetFormattedData(dataFile, timeVariable);

         exportData.scs.closeAndDispose();
         exportData.simulation = null;
         /*
          * try { exportM3Data.finalize(); } catch (Throwable e) {
          * e.printStackTrace(); } try { exportM3Data.simulation.join();
          * exportM3Data.scs.closeAndDispose();
          * exportM3Data.scs.stopSimulationThread();
          * exportM3Data.simulation.stop(); } catch (Throwable e) {
          * System.out.print("Join interrupted\n"); return; }
          */
      }

      private void setCamera(AtlasMultiDataExporter exportM3Data, double[] cameraParameters)
      {
         double hour = cameraParameters[0];
         double radius = cameraParameters[1];
         double height = cameraParameters[2];

         RigidBodyTransform ret = new RigidBodyTransform();
         Vector3D cameraFix = new Vector3D();
         double angle = Math.PI / 2 + ((hour) * Math.PI / 6);
         Vector3D cameraPosition = new Vector3D(radius * Math.sin(angle), radius * Math.cos(angle), height);

         Robot[] robot = exportM3Data.scs.getRobots();
         ArrayList<Joint> joint = robot[0].getRootJoints();
         joint.get(0).getTransformToWorld(ret);
         ret.transform(cameraPosition);
         ret.getTranslation(cameraFix);

         CameraConfiguration cameraConfiguration = new CameraConfiguration("testCamera");
         cameraConfiguration.setCameraFix(cameraFix);
         cameraConfiguration.setCameraPosition(cameraPosition);
         cameraConfiguration.setCameraTracking(true, true, true, false);
         exportM3Data.scs.setupCamera(cameraConfiguration);
         exportM3Data.scs.selectCamera("testCamera");
      }

      private void gotoPoint(int newValue, AtlasMultiDataExporter exportM3Data)
      {
         if (!exportM3Data.scs.isSimulating())
         {
            exportM3Data.robot.seek(newValue);

            try
            {
               exportM3Data.scs.simulateOneRecordStepNow();
               exportM3Data.scs.setInPoint();
            }
            catch (UnreasonableAccelerationException e)
            {
               e.printStackTrace();
            }

            exportM3Data.players.indexChanged(0, 0);
         }
      }

      private void writeSpreadsheetFormattedData(File chosenFile, String timeVariable)
      {
         PrintTools.info(this, "Writing Data File " + chosenFile.getAbsolutePath());

         DataBuffer dataBuffer = exportData.scs.getDataBuffer();

         ArrayList<YoVariable<?>> varsYo = exportData.scs.getVars(vars, new String[0]);
         writeSpreadsheetFormattedData(chosenFile, dataBuffer, varsYo, timeVariable);
      }

      private void exportVideo(final AtlasMultiDataExporter exportM3Data, String savePath, String fileName)
      {
         if (exportM3Data.players != null)
         {
            long[] timestamps = getInAndOut(exportM3Data);
            if (timestamps.length != 2)
            {
               return;
            }

            final long startTimestamp = timestamps[0];
            final long endTimestamp = timestamps[1];

            final File selectedFile = new File(savePath + File.separator + fileName + ".mov");

            new Thread()
            {
               @Override
               public void run()
               {
                  exportM3Data.players.exportCurrentVideo(selectedFile, startTimestamp, endTimestamp);
               }
            }.start();
         }
      }

      private void switchVideoUpdate(AtlasMultiDataExporter exportM3Data, String videoName)
      {
         if (exportM3Data.players != null)
         {
            if (videoName.equals("none") || !showCameraVideo)
            {
               exportM3Data.players.setActivePlayer(null);
            }
            else
            {
               exportM3Data.players.setActivePlayer(videoName);
            }
         }

      }

      private void writeSpreadsheetFormattedData(File chosenFile, DataBuffer dataBuffer, ArrayList<? extends YoVariable<?>> vars, String timeVariable)
      {
         ArrayList<DataBufferEntry> entries = dataBuffer.getEntries();

         try
         {
            PrintStream printStream;

            printStream = new PrintStream(new BufferedOutputStream(new FileOutputStream(chosenFile)));
            int bufferLength = dataBuffer.getBufferInOutLength();

            String varnamesToWrite[] = new String[vars.size()];

            double[][] dataToWrite = new double[vars.size()][];

            for (int i = 0; i < entries.size(); i++)
            {
               DataBufferEntry entry = entries.get(i);
               YoVariable<?> variable = entry.getVariable();
               entry.getData();

               if (vars.contains(variable))
               {
                  varnamesToWrite[vars.indexOf(variable)] = entry.getVariable().getName();
                  double[] allData = entry.getData();
                  double[] data = getWindowedData(dataBuffer.getInPoint(), allData, bufferLength);
                  if (entry.getVariable().getName().equals(timeVariable))
                  {
                     double initialTime = data[0];
                     for (int j = 0; j < data.length; j++)
                     {
                        data[j] -= initialTime;
                     }
                  }

                  dataToWrite[vars.indexOf(variable)] = data;
               }
            }

            for (int i = 0; i < varnamesToWrite.length; i++)
            {
               printStream.print(varnamesToWrite[i]);
               if (i < varnamesToWrite.length - 1)
                  printStream.print(",");
               else
                  printStream.println("");
            }

            for (int j = 0; j < bufferLength; j++)
            {
               for (int i = 0; i < dataToWrite.length; i++)
               {
                  double[] data = dataToWrite[i];
                  printStream.print(data[j]);
                  if (i < dataToWrite.length - 1)
                     printStream.print(",");
                  else
                     printStream.println("");
               }
            }

            printStream.close();
            this.exportFinish = true;
         }
         catch (IOException e)
         {
         }
      }

      private long[] getInAndOut(AtlasMultiDataExporter exportM3Data)
      {
         if (exportM3Data.scs.isSimulating())
         {
            exportM3Data.scs.stop();
         }

         exportM3Data.scs.gotoInPointNow();
         long startTimestamp = exportM3Data.players.getCurrentTimestamp();
         exportM3Data.scs.gotoOutPointNow();
         long endTimestamp = exportM3Data.players.getCurrentTimestamp();

         if (startTimestamp > endTimestamp)
         {
            JOptionPane.showMessageDialog(null, "startTimestamp > endTimestamp. Please set the in-point and out-point correctly", "Timestmap error", JOptionPane.ERROR_MESSAGE);
            return new long[0];
         }

         return new long[] { startTimestamp, endTimestamp };
      }

      private double[] getWindowedData(int in, double[] allData, int bufferLength)
      {
         double[] ret = new double[bufferLength];
         int n = in;

         for (int i = 0; i < bufferLength; i++)
         {
            ret[i] = allData[n];
            n++;
            if (n >= allData.length)
               n = 0;
         }

         return ret;
      }
   }
}
