package us.ihmc.robotDataCommunication.logger;

import java.awt.BorderLayout;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.channels.FileChannel;

import javax.swing.Action;
import javax.swing.ActionMap;
import javax.swing.JFileChooser;
import javax.swing.JTable;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.robotDataCommunication.VisualizerUtils;
import us.ihmc.robotDataCommunication.YoVariableHandshakeParser;
import us.ihmc.utilities.SwingUtils;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class YoVariableLogVisualizer
{
   private static final String defaultLogReadingDirectory = "/media/remote/RobotLogs";

   private final SDFJointNameMap jointNameMap;
   private final GeneralizedSDFRobotModel generalizedSDFRobotModel;
   private final String timeVariableName;
   private final int bufferSize;
   protected final SimulationConstructionSet scs;

   public YoVariableLogVisualizer(GeneralizedSDFRobotModel generalizedSDFRobotModel, SDFJointNameMap jointNameMap, String timeVariableName, 
         int bufferSize, boolean showOverheadView, File logFile) throws IOException
   {
      this.bufferSize = bufferSize;
      this.generalizedSDFRobotModel = generalizedSDFRobotModel;
      this.jointNameMap = jointNameMap;
      this.timeVariableName = timeVariableName;

      if (logFile == null)
      {
         final JFileChooser fileChooser = new JFileChooser(new File(defaultLogReadingDirectory));
         sortByDateHack(fileChooser);  
         
         fileChooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
         int returnValue = fileChooser.showOpenDialog(null);
         if(returnValue == JFileChooser.APPROVE_OPTION)
         {
            logFile = fileChooser.getSelectedFile();
         }
         else
         {
            System.err.println("No file selected, closing.");
         }
      }
      
      if(logFile != null)
      {
         System.out.println("loading log from folder:" + logFile);
         scs= new SimulationConstructionSet(true, bufferSize);
         readLogFile(logFile, showOverheadView); 
      }
      else
      {
         scs=null;
      }
      
   }

   private void sortByDateHack(final JFileChooser fileChooser)
   {
      
      // Found this hack on http://stackoverflow.com/questions/16429779/start-a-jfilechooser-with-files-ordered-by-date
      ActionMap actionMap = fileChooser.getActionMap();
      Action details = actionMap.get("viewTypeDetails");
      if(details != null)
      {
         details.actionPerformed(null);
   
         //  Find the JTable on the file chooser panel and manually do the sort
   
         JTable table = SwingUtils.getDescendantsOfType(JTable.class, fileChooser).get(0);
         table.getRowSorter().toggleSortOrder(2); table.getRowSorter().toggleSortOrder(2);
      }
      else
      {
         System.err.println("sort by datetime doesn't work known on OSX ... bail out");
      }
   }

   private void readLogFile(File selectedFile, boolean showOverheadView) throws IOException
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
      
      
      scs.setTimeVariableName(timeVariableName);
      YoVariableLogPlaybackRobot robot = new YoVariableLogPlaybackRobot(generalizedSDFRobotModel,jointNameMap, parser.getJointStates(), parser.getYoVariablesList(), logChannel, scs);
      
      double dt = parser.getDt();
      System.out.println(getClass().getSimpleName()+ ": dt set to " + dt);
      scs.setDT(dt, 1);
      scs.setPlaybackDesiredFrameRate(0.04);
      
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = parser.getDynamicGraphicObjectsListRegistry();
      dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);
      VisualizerUtils.createOverheadPlotter(dynamicGraphicObjectsListRegistry, scs, showOverheadView);
      scs.getRootRegistry().addChild(parser.getRootRegistry());
      scs.setGroundVisible(false);
      
      MultiVideoDataPlayer players = null;
      try
      {
         
         players = new MultiVideoDataPlayer(selectedFile, logProperties, robot.getTimestamp());
         
         scs.attachPlaybackListener(players);
         scs.attachSimulationRewoundListener(players);

      }
      catch(Exception e)
      {
         System.err.println("Couldn't load video file!");
         e.printStackTrace();
      }
 
      YoVariableLogCropper yoVariableLogCropper = new YoVariableLogCropper(players, selectedFile, logProperties);
      
      scs.getJFrame().setTitle(this.getClass().getSimpleName() + " - " + selectedFile);
      scs.getStandardSimulationGUI().addJComponentToMainPanel( new YoVariableLogVisualizerGUI(players, robot, yoVariableLogCropper, scs), BorderLayout.SOUTH);
   }
   
   public void run()
   {
      new Thread(scs).start();
   }
}
