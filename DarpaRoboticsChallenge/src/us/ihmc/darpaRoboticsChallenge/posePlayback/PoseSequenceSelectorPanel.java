package us.ihmc.darpaRoboticsChallenge.posePlayback;

import java.awt.Dimension;
import java.awt.GridLayout;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFileChooser;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.table.AbstractTableModel;

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
   
   private final JTable table;
   private final ScriptEditorTableModel tableModel;
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

       tableModel = new ScriptEditorTableModel();
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
      
      PosePlaybackRobotPose selectedPose = sequence.getPoseSequence().get(selectedRow);
      
      selectedPose.setRobotAtPose(sdfRobot);
   }
   
   private void deleteRow(int row)
   {
      ArrayList<PosePlaybackRobotPose> poseSequence = sequence.getPoseSequence();
      poseSequence.remove(row);
   }

   private class ScriptEditorTableModel extends AbstractTableModel 
   {
      private int numberOfRows = 1;
  
       private String[] columnNames = new String[]
       {
             "#", "sy", "sp", "sr", "neck", "lhy", "lhr", "lhp", "lk", "lap", "lar", "rhy", "rhr", 
             "rhp", "rk", "rap", "rar", "lsp", "lsr", "lep", "ler", "lwp", "lwr", 
             "rsp", "rsr", "rep", "rer", "rwp", "rwr", "pause"
       };

       private List<Object[]> data = new ArrayList<Object[]>();

       public ScriptEditorTableModel()
       {
         data.add(new Object[]
            {
               0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            });
       }

       public int getColumnCount() 
       {
           return columnNames.length;
       }

       public int getRowCount() 
       {
           return data.size();
       }
       
       public String getColumnName(int col) 
       {
           return columnNames[col];
       }

       public Object getValueAt(int row, int col) 
       {
           return data.get(row)[col];
       }

       public boolean isCellEditable(int row, int col) 
       {
          return true;
       }
       
       public void addRow(Object[] row)
       {
          numberOfRows++;
          data.add(row);
          fireTableDataChanged();
       }   
   }
      
   private void updateTableBasedOnPoseSequence()
   {
      tableModel.data.clear();
      
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
      
      System.out.println(table.getRowCount());
   }
}
