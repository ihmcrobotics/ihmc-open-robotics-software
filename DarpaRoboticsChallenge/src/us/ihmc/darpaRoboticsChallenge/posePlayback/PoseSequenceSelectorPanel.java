package us.ihmc.darpaRoboticsChallenge.posePlayback;

import java.awt.Dimension;
import java.awt.GridLayout;
import java.io.File;
import java.util.ArrayList;

import javax.swing.JFileChooser;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.table.DefaultTableModel;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedSensorReader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.darpaRoboticsChallenge.environment.VRCTask;
import us.ihmc.darpaRoboticsChallenge.environment.VRCTaskName;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.ModularRobotController;

public class PoseSequenceSelectorPanel extends JPanel 
{
   private final YoVariableRegistry registry = new YoVariableRegistry("PoseSequenceGUI");
   private final PosePlaybackAllJointsController posePlaybackController;
   private final SDFRobot sdfRobot;
   private final DRCRobotMidiSliderBoardPositionManipulation sliderBoard;
   
   private final JTable table;
   private final DefaultTableModel tableModel;
   private PosePlaybackRobotPoseSequence sequence = new PosePlaybackRobotPoseSequence();
   
   public PoseSequenceSelectorPanel() 
   {
       super(new GridLayout(1,0));
       
       posePlaybackController = new PosePlaybackAllJointsController(registry);
       VRCTask vrcTask = new VRCTask(VRCTaskName.ONLY_VEHICLE);
       SDFFullRobotModel fullRobotModel = vrcTask.getFullRobotModelFactory().create();
       
       sdfRobot = vrcTask.getRobot();
       ReferenceFrames referenceFrames = new ReferenceFrames(fullRobotModel, vrcTask.getJointMap(), vrcTask.getJointMap().getAnkleHeight());
       SDFPerfectSimulatedSensorReader reader = new SDFPerfectSimulatedSensorReader(sdfRobot, fullRobotModel, referenceFrames);
       ModularRobotController controller = new ModularRobotController("Reader");
       controller.setRawSensorReader(reader);
       
       SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);
       scs.addYoVariableRegistry(registry);
       scs.startOnAThread();
       sliderBoard = new DRCRobotMidiSliderBoardPositionManipulation(scs);

       String[] columnNames = new String[]
       {
             "#", "sy", "sp", "sr", "neck", "lhy", "lhr", "lhp", "lk", "lap", "lar", "rhy", "rhr", 
             "rhp", "rk", "rap", "rar", "lsp", "lsr", "lep", "ler", "lwp", "lwr", 
             "rsp", "rsr", "rep", "rer", "rwp", "rwr", "pause"
       };
       tableModel = new DefaultTableModel(columnNames, 0); // new ScriptEditorTableModel();
       table = new JTable(tableModel);

       table.getColumnModel().getColumn(0).setPreferredWidth(40);
       for(int i = 1; i < 29; i++)
       {
          table.getColumnModel().getColumn(i).setPreferredWidth(40);
       }
       table.getColumnModel().getColumn(29).setPreferredWidth(40);
       
       table.setPreferredScrollableViewportSize(new Dimension(500, 200));
       table.setFillsViewportHeight(true);

       JScrollPane scrollPane = new JScrollPane(table);
            
       add(scrollPane);
   }
   
   public void setValueAt(Object object, int row, int col) 
   {
      tableModel.setValueAt(object, row, col);
   }
   
   public void addSequence()
   {
      JFileChooser chooser = new JFileChooser(new File("PoseSequences"));
      int approveOption = chooser.showOpenDialog(null);

      if (approveOption != JFileChooser.APPROVE_OPTION)
      {
         System.err.println("Can not load selected file :" + chooser.getName());
         return;
      }

      File selectedFile = chooser.getSelectedFile();

      sequence.appendFromFile(selectedFile);
      updateTableBasedOnPoseSequence();
   }
   
   public void deleteSelectedRows()
   {
      int[] selectedRows = table.getSelectedRows();
      int numberOfRemovedRows = 0;
      
      for(int row : selectedRows)
      {
         deleteRow(row - numberOfRemovedRows);
         numberOfRemovedRows++;
      }
      
      updateTableBasedOnPoseSequence();
   }
   
   public void updateSCS()
   {
      int selectedRow = table.getSelectedRow();
      if(selectedRow == -1)
         return;
      
      PosePlaybackRobotPose selectedPose = sequence.getPoseSequence().get(selectedRow);
      
      selectedPose.setRobotAtPose(sdfRobot);
   }
   
   public void setRowWithSlider()
   {
      int selectedRow = table.getSelectedRow();
      if(selectedRow == -1)
         return;
      
      sequence.getPoseSequence().set(selectedRow, new PosePlaybackRobotPose(sdfRobot));
   }
   
   public void save()
   {
      sequence.promptWriteToFile();
   }
   
   private void deleteRow(int row)
   {
      ArrayList<PosePlaybackRobotPose> poseSequence = sequence.getPoseSequence();
      poseSequence.remove(row);
   }
      
   private void updateTableBasedOnPoseSequence()
   {      
      tableModel.setRowCount(0);
      
      ArrayList<PosePlaybackRobotPose> poseSequence = sequence.getPoseSequence();
      for(int i = 0; i < poseSequence.size(); i++)
      {
         Object[] row = new Object[30];
         row[0] = i;
         
         PosePlaybackRobotPose pose = poseSequence.get(i);
         double[] jointAngles = pose.getJointAngles();
         for(int j = 0; j < jointAngles.length; j++)
         {
            row[j + 1] = jointAngles[j];
         }
         
         row[jointAngles.length + 1] = pose.getPlayBackDelayBeforePose();
         
         tableModel.addRow(row);
      }      
   }
}
