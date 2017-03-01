package us.ihmc.avatar.posePlayback;

import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.File;
import java.util.ArrayList;

import javax.swing.JFileChooser;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.table.DefaultTableModel;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class PoseSequenceSelectorPanel extends JPanel
{
   private static final long serialVersionUID = 7616401668436177628L;

   private final YoVariableRegistry registry;
   private final HumanoidFloatingRootJointRobot sdfRobot;
   private final FullHumanoidRobotModel fullRobotModel;
   private final DRCRobotMidiSliderBoardPositionManipulation sliderBoard;

   private final JTable table;
   private final DefaultTableModel tableModel;
   private final PlaybackPoseSequence sequence;

   public PoseSequenceSelectorPanel(DRCRobotModel robotModel)
   {
      super(new GridLayout(1, 0));
      registry = new YoVariableRegistry("PoseSequenceGUI");


      fullRobotModel = robotModel.createFullRobotModel();
      sdfRobot = robotModel.createHumanoidFloatingRootJointRobot(false);

      sequence = new PlaybackPoseSequence(fullRobotModel);
      
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      SDFPerfectSimulatedSensorReader reader = new SDFPerfectSimulatedSensorReader(sdfRobot, fullRobotModel, referenceFrames);
      ModularRobotController controller = new ModularRobotController("Reader");
      controller.setRawSensorReader(reader);

      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);
      scs.addYoVariableRegistry(registry);
      scs.startOnAThread();
      sliderBoard = new DRCRobotMidiSliderBoardPositionManipulation(scs, sdfRobot, fullRobotModel, null);

      String[] columnNames = new String[] { "#", "sy", "sp", "sr", "neck", "lhy", "lhr", "lhp", "lk", "lap", "lar", "rhy", "rhr", "rhp", "rk", "rap", "rar",
            "lsp", "lsr", "lep", "ler", "lwp", "lwr", "rsp", "rsr", "rep", "rer", "rwp", "rwr", "pause" };
      tableModel = new DefaultTableModel(columnNames, 0); // new ScriptEditorTableModel();
      table = new JTable(tableModel);

      tableInit();
   }

   private void tableInit()
   {
      table.addKeyListener(new KeyListener()
      {
         public void keyPressed(KeyEvent e)
         {
            if (e.getKeyChar() == KeyEvent.VK_ENTER)
               updateSCS();
         }

         public void keyReleased(KeyEvent e)
         {
         }

         public void keyTyped(KeyEvent e)
         {
         }
      });

      table.getColumnModel().getColumn(0).setPreferredWidth(40);
      for (int i = 1; i < 29; i++)
      {
         table.getColumnModel().getColumn(i).setPreferredWidth(40);
      }
      table.getColumnModel().getColumn(29).setPreferredWidth(40);

      table.setPreferredScrollableViewportSize(new Dimension(500, 200));
      table.setFillsViewportHeight(true);

      JScrollPane scrollPane = new JScrollPane(table);

      add(scrollPane);
   }

   public PoseSequenceSelectorPanel(YoVariableRegistry registry, HumanoidFloatingRootJointRobot sdfRobot, FullHumanoidRobotModel fullRobotModel, DRCRobotMidiSliderBoardPositionManipulation sliderBoard)
   {
      super(new GridLayout(1, 0));

      this.registry = registry;
      this.sdfRobot = sdfRobot;
      this.fullRobotModel = fullRobotModel;
      this.sliderBoard = sliderBoard;

      sequence = new PlaybackPoseSequence(fullRobotModel);

      String[] columnNames = new String[] { "#", "sy", "sp", "sr", "neck", "lhy", "lhr", "lhp", "lk", "lap", "lar", "rhy", "rhr", "rhp", "rk", "rap", "rar",
            "lsp", "lsr", "lep", "ler", "lwp", "lwr", "rsp", "rsr", "rep", "rer", "rwp", "rwr", "pause" };
      tableModel = new DefaultTableModel(columnNames, 0); // new ScriptEditorTableModel();
      table = new JTable(tableModel);

      tableInit();
   }

   public void setValueAt(Object object, int row, int col)
   {
      tableModel.setValueAt(object, row, col);
   }

   public void addSequenceFromFile()
   {
      File selectedFile = selectFile();

      if (selectedFile != null)
      {
         PlaybackPoseSequenceReader.appendFromFile(sequence, selectedFile);
         updateTableBasedOnPoseSequence();
      }
   }

   public void newSequenceFromFile()
   {
      File selectedFile = selectFile();

      if (selectedFile != null)
      {
         sequence.clear();
         PlaybackPoseSequenceReader.appendFromFile(sequence, selectedFile);
         updateTableBasedOnPoseSequence();
      }
   }

   private File selectFile()
   {
      JFileChooser chooser = new JFileChooser(new File("PoseSequences"));
      int approveOption = chooser.showOpenDialog(null);

      File selectedFile;
      if (approveOption != JFileChooser.APPROVE_OPTION)
      {
         System.err.println("Can not load selected file :" + chooser.getName());
         selectedFile = null;
      }
      else
         selectedFile = chooser.getSelectedFile();

      return selectedFile;
   }

   public void addSequence(PlaybackPoseSequence seq)
   {
      for (PlaybackPose pose : seq.getPoseSequence())
      {
         sequence.addPose(pose);
      }
      updateTableBasedOnPoseSequence();
   }

   public void setSequence(PlaybackPoseSequence seq)
   {
      sequence.clear();
      addSequence(seq);
   }

   public void deleteSelectedRows()
   {
      updatePoseSequenceBasedOnTable();

      int[] selectedRows = table.getSelectedRows();
      int numberOfRemovedRows = 0;

      for (int row : selectedRows)
      {
         sequence.getPoseSequence().remove(row - numberOfRemovedRows);
         numberOfRemovedRows++;
      }

      updateTableBasedOnPoseSequence();
   }

   public void updateSCS()
   {
      updatePoseSequenceBasedOnTable();

      int selectedRow = table.getSelectedRow();
      if (selectedRow == -1)
         return;

      PlaybackPose selectedPose = sequence.getPoseSequence().get(selectedRow);

      selectedPose.setRobotAtPose(sdfRobot);
   }

   public void setRowWithSlider()
   {
      int selectedRow = table.getSelectedRow();
      if (selectedRow == -1)
         return;

      PlaybackPose pose = new PlaybackPose(fullRobotModel, sdfRobot);
      pose.setPlaybackDelayBeforePose(getTimeDelayFromRow(selectedRow));
      sequence.getPoseSequence().set(selectedRow, pose);
      updateTableBasedOnPoseSequence();
   }

   public void save()
   {
      updatePoseSequenceBasedOnTable();
      PlaybackPoseSequenceWriter.promptWriteToFile(sequence);
   }

   private void updateTableBasedOnPoseSequence()
   {
      tableModel.setRowCount(0);

      ArrayList<PlaybackPose> poseSequence = sequence.getPoseSequence();
      for (int i = 0; i < poseSequence.size(); i++)
      {
         Object[] row = new Object[30];
         row[0] = i;

         PlaybackPose pose = poseSequence.get(i);
         double[] jointAngles = pose.getJointAngles();
         for (int j = 0; j < jointAngles.length; j++)
         {
            row[j + 1] = jointAngles[j];
         }

         row[jointAngles.length + 1] = pose.getPlayBackDelayBeforePose();

         tableModel.addRow(row);
      }
   }

   private void updatePoseSequenceBasedOnTable()
   {
      throw new RuntimeException("Please implement me!");
      
//      sequence.clear();
//
//      for (int i = 0; i < tableModel.getRowCount(); i++)
//      {
//         sequence.addPose(getJointAnglesFromRow(i), getTimeDelayFromRow(i));
//      }
   }

   private double[] getJointAnglesFromRow(int row)
   {
      double[] jointAngles = new double[28];

      for (int i = 0; i < 28; i++)
      {
         try
         {
            jointAngles[i] = Double.parseDouble((String) tableModel.getValueAt(row, i + 1));
         }
         catch (ClassCastException e)
         {
            jointAngles[i] = (Double) tableModel.getValueAt(row, i + 1);
         }
      }

      return jointAngles;
   }

   private double getTimeDelayFromRow(int row)
   {
      double timeDelay;
      try
      {
         timeDelay = Double.parseDouble((String) tableModel.getValueAt(row, 29));
      }
      catch (ClassCastException e)
      {
         timeDelay = (Double) tableModel.getValueAt(row, 29);
      }

      return timeDelay;
   }

   public void copyAndInsertRow()
   {
      updatePoseSequenceBasedOnTable();

      int[] selectedRows = table.getSelectedRows();

      if (selectedRows.length == 0)
      {
         System.out.println("No row selected to copy.");
         return;
      }
      int row = selectedRows[0];
      sequence.getPoseSequence().add(row, sequence.getPose(row).copy());

      updateTableBasedOnPoseSequence();
   }

   public void insertInterpolation()
   {
      throw new RuntimeException("Please implement me again!");

//      updatePoseSequenceBasedOnTable();
//
//      int[] selectedRows = table.getSelectedRows();
//
//      if (selectedRows.length == 0)
//      {
//         System.out.println("No row selected for interpolation.");
//         return;
//      }
//      int row = selectedRows[0];
//      double[] pose1 = sequence.getPose(row).getJointAngles();
//      double[] pose2 = sequence.getPose(row + 1).getJointAngles();
//      double del1 = sequence.getPose(row).getPlayBackDelayBeforePose();
//      double del2 = sequence.getPose(row + 1).getPlayBackDelayBeforePose();
//      double delInterp = (del1 + del2) / 2;
//      double[] poseInterp = new double[ROSAtlasJointMap.numberOfJoints];
//      for (int i = 0; i < ROSAtlasJointMap.numberOfJoints; i++)
//      {
//         poseInterp[i] = (pose1[i] + pose2[i]) / 2;
//      }
//
//      PosePlaybackRobotPose interpPose = new PosePlaybackRobotPose(poseInterp, delInterp);
//
//      sequence.getPoseSequence().add(row + 1, interpPose);
//
//      updateTableBasedOnPoseSequence();
   }

   public void switchSideDependentValues()
   {
      throw new RuntimeException("Please implement me again!");
//      int[] selectedRows = table.getSelectedRows();
//
//      if (selectedRows.length != 0)
//      {
//         for (int row : selectedRows)
//         {
//            PosePlaybackRobotPose pose = sequence.getPoseSequence().get(row);
//            pose.switchSideDependentValues();
//         }
//      }
//
//      updateTableBasedOnPoseSequence();
   }
}
