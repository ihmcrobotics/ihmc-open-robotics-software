package us.ihmc.robotDataCommunication.logger;

import java.awt.BorderLayout;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.channels.FileChannel;

import javax.swing.JFileChooser;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.robotDataCommunication.VisualizerUtils;
import us.ihmc.robotDataCommunication.YoVariableHandshakeParser;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class YoVariableLogVisualizer
{
   private final JaxbSDFLoader loader;
   private final SDFJointNameMap jointNameMap;
   
   public YoVariableLogVisualizer(JaxbSDFLoader loader, SDFJointNameMap jointNameMap) throws IOException
   {
      this.loader = loader;
      this.jointNameMap = jointNameMap;
      
      final JFileChooser fileChooser = new JFileChooser(new File(YoVariableLoggerOptions.defaultLogDirectory));
      fileChooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
      int returnValue = fileChooser.showOpenDialog(null);
      if(returnValue == JFileChooser.APPROVE_OPTION)
      {
         readLogFile(fileChooser.getSelectedFile());
      }
      else
      {
         System.err.println("No file selected, closing.");
      }
      
   }

   private void readLogFile(File selectedFile) throws IOException
   {
      LogPropertiesReader logProperties = new LogPropertiesReader(new File(selectedFile, YoVariableLoggerListener.propertyFile));
      File handshake = new File(selectedFile, logProperties.getHandshakeFile());
      if(!handshake.exists())
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
      if(!logdata.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getVariableDataFile());
      }
      @SuppressWarnings("resource")
      final FileChannel logChannel = new FileInputStream(logdata).getChannel();
      
      
      
      SimulationConstructionSet scs = new SimulationConstructionSet();
      YoVariableLogPlaybackRobot robot = new YoVariableLogPlaybackRobot(loader.getGeneralizedSDFRobotModel(jointNameMap.getModelName()),jointNameMap, parser.getJointStates(), parser.getYoVariablesList(), logChannel, scs);
      scs.setDT(parser.getDt(), 1);
      scs.setPlaybackDesiredFrameRate(0.04);
      
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = parser.getDynamicGraphicObjectsListRegistry();
      dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);
      VisualizerUtils.createOverheadPlotter(dynamicGraphicObjectsListRegistry, scs);
      scs.getRootRegistry().addChild(parser.getRootRegistry());
      scs.setGroundVisible(false);
      
      VideoDataPlayer player = new VideoDataPlayer(selectedFile, logProperties, robot.getTimestamp());
      scs.attachPlaybackListener(player);
      scs.attachSimulationRewoundListener(player);
      
      scs.getStandardSimulationGUI().addJComponentToMainPanel( new YoVariableLogVisualizerGUI(player, robot, scs), BorderLayout.SOUTH);

      
     
      
      
      
      new Thread(scs).start();
   }
   
   
}
