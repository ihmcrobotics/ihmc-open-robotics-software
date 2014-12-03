package us.ihmc.robotDataCommunication.logger;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.channels.FileChannel;

import javax.swing.JLabel;
import javax.swing.JTextField;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.plotting.Plotter;
import us.ihmc.robotDataCommunication.VisualizerUtils;
import us.ihmc.robotDataCommunication.YoVariableHandshakeParser;
import us.ihmc.robotDataCommunication.logger.util.FileSelectionDialog;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.plotting.SimulationOverheadPlotter;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class YoVariableLogVisualizer
{
   private final SDFJointNameMap jointNameMap;
   private final GeneralizedSDFRobotModel generalizedSDFRobotModel;
   private final String timeVariableName;
   protected final SimulationConstructionSet scs;
   private YoVariableLogPlaybackRobot robot;
   private SimulationOverheadPlotter plotter;

   public YoVariableLogVisualizer(GeneralizedSDFRobotModel generalizedSDFRobotModel, SDFJointNameMap jointNameMap, String timeVariableName) throws IOException
   {
      this(generalizedSDFRobotModel, jointNameMap, timeVariableName, 32768, false, null);
   }

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
      robot = new YoVariableLogPlaybackRobot(generalizedSDFRobotModel, jointNameMap, parser.getJointStates(), parser.getYoVariablesList(), logChannel, scs);

      double dt = parser.getDt();
      System.out.println(getClass().getSimpleName() + ": dt set to " + dt);
      scs.setDT(dt, 1);
      scs.setPlaybackDesiredFrameRate(0.04);

      YoGraphicsListRegistry yoGraphicsListRegistry = parser.getDynamicGraphicObjectsListRegistry();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      plotter = VisualizerUtils.createOverheadPlotter(scs, showOverheadView, yoGraphicsListRegistry);
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
      YoVariableLogVisualizerGUI gui = new YoVariableLogVisualizerGUI(selectedFile, players, robot, yoVariableLogCropper, scs);
      scs.getStandardSimulationGUI().addJComponentToMainPanel(gui, BorderLayout.SOUTH);
      
      
      setupReadEveryNTicksTextField();
   }

   private void setupReadEveryNTicksTextField()
   {
      JLabel label = new JLabel("ReadEveryNTicks");
      scs.addJLable(label);
      
      String everyNTicksString = Integer.toString(robot.getReadEveryNTicks());
      final JTextField textField = new JTextField(everyNTicksString, 3);
      textField.addActionListener(new ActionListener(){

         @Override
         public void actionPerformed(ActionEvent e)
         {
            try
            {
               int readEveryNTicks = Integer.parseInt(textField.getText());
               robot.setReadEveryNTicks(readEveryNTicks);
            }
            catch (NumberFormatException exception)
            {
            }
            
            String everyNTicksString = Integer.toString(robot.getReadEveryNTicks());
            textField.setText(everyNTicksString);
            textField.getParent().requestFocus();
         }});
      
      scs.addTextField(textField);
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return scs;
   }

   public SDFRobot getSDFRobot()
   {
      return robot;
   }

   public Plotter getPlotter()
   {
      return plotter.getPlotter();
   }

   public void run()
   {
      new Thread(scs).start();
   }
}
