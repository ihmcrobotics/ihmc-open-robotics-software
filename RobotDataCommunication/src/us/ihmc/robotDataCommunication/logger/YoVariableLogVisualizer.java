package us.ihmc.robotDataCommunication.logger;

import java.awt.BorderLayout;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.channels.FileChannel;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.robotDataCommunication.VisualizerUtils;
import us.ihmc.robotDataCommunication.YoVariableHandshakeParser;
import us.ihmc.robotDataCommunication.logger.util.FileSelectionDialog;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;

public class YoVariableLogVisualizer
{
   private final SDFJointNameMap jointNameMap;
   private final GeneralizedSDFRobotModel generalizedSDFRobotModel;
   private final String timeVariableName;
   protected final SimulationConstructionSet scs;

   public YoVariableLogVisualizer(GeneralizedSDFRobotModel generalizedSDFRobotModel, SDFJointNameMap jointNameMap, String timeVariableName, int bufferSize,
         boolean showOverheadView, File logFile) throws IOException
   {
      this.generalizedSDFRobotModel = generalizedSDFRobotModel;
      this.jointNameMap = jointNameMap;
      this.timeVariableName = timeVariableName;

      if (logFile == null)
      {
         logFile = FileSelectionDialog.loadDirectoryWithFileNamed("robotData.log");
      }

      if (logFile != null)
      {
         System.out.println("loading log from folder:" + logFile);
         scs = new SimulationConstructionSet(true, bufferSize);
         readLogFile(logFile, showOverheadView);
      }
      else
      {
         scs = null;
      }

   }

   private void readLogFile(File selectedFile, boolean showOverheadView) throws IOException
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

      YoVariableHandshakeParser parser = new YoVariableHandshakeParser(null, "logged", true);
      parser.parseFrom(handshakeData);

      File logdata = new File(selectedFile, logProperties.getVariableDataFile());
      if (!logdata.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getVariableDataFile());
      }
      final FileChannel logChannel = new FileInputStream(logdata).getChannel();

      scs.setTimeVariableName(timeVariableName);
      YoVariableLogPlaybackRobot robot = new YoVariableLogPlaybackRobot(generalizedSDFRobotModel, jointNameMap, parser.getJointStates(),
            parser.getYoVariablesList(), logChannel, scs);

      double dt = parser.getDt();
      System.out.println(getClass().getSimpleName() + ": dt set to " + dt);
      scs.setDT(dt, 1);
      scs.setPlaybackDesiredFrameRate(0.04);

      YoGraphicsListRegistry yoGraphicsListRegistry = parser.getDynamicGraphicObjectsListRegistry();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      VisualizerUtils.createOverheadPlotter(scs, showOverheadView, yoGraphicsListRegistry);
      scs.getRootRegistry().addChild(parser.getRootRegistry());
      scs.setGroundVisible(false);

      MultiVideoDataPlayer players = null;
      try
      {
         players = new MultiVideoDataPlayer(selectedFile, logProperties, robot.getTimestamp());

         scs.attachPlaybackListener(players);
         scs.attachSimulationRewoundListener(players);

      }
      catch (Exception e)
      {
         System.err.println("Couldn't load video file!");
         e.printStackTrace();
      }

      YoVariableLogCropper yoVariableLogCropper = new YoVariableLogCropper(players, selectedFile, logProperties);

      scs.getJFrame().setTitle(this.getClass().getSimpleName() + " - " + selectedFile);
      scs.getStandardSimulationGUI().addJComponentToMainPanel(new YoVariableLogVisualizerGUI(selectedFile, players, robot, yoVariableLogCropper, scs),
            BorderLayout.SOUTH);
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return scs;
   }

   public void run()
   {
      new Thread(scs).start();
   }
}
