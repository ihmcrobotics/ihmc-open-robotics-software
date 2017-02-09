package us.ihmc.avatar.posePlayback;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;

public class PoseSequenceEditorGUI extends JFrame
{
   private static final long serialVersionUID = 8890174114593028871L;
   private final PoseSequenceSelectorPanel poseSequenceSelectorPanel;
   private final ButtonPanel buttonPanel;

   public PoseSequenceEditorGUI(DRCRobotModel robotModel)
   {
      super("Pose sequence editor");
      setSize(1400, 600);
      poseSequenceSelectorPanel = new PoseSequenceSelectorPanel(robotModel);
      buttonPanel = new ButtonPanel();

      buttonPanelInit();
   }

   public PoseSequenceEditorGUI(YoVariableRegistry registry, HumanoidFloatingRootJointRobot sdfRobot, FullHumanoidRobotModel fullRobotModel, DRCRobotMidiSliderBoardPositionManipulation sliderBoard)
   {
      super("Pose sequence editor");
      setSize(1400, 600);
      poseSequenceSelectorPanel = new PoseSequenceSelectorPanel(registry, sdfRobot, fullRobotModel, sliderBoard);
      buttonPanel = new ButtonPanel();

      buttonPanelInit();
   }

   private void buttonPanelInit()
   {
      getContentPane().add(new JointNameKey(), BorderLayout.NORTH);
      getContentPane().add(poseSequenceSelectorPanel, BorderLayout.CENTER);
      getContentPane().add(buttonPanel, BorderLayout.SOUTH);
   }

   private class ButtonPanel extends JPanel implements ActionListener
   {
      private static final long serialVersionUID = 5462601984021392064L;
      private JButton selectNewPoseSequence, selectPoseSequence, deleteRow, updateSCS, setRowWithSlider, save, switchSideDependentValues, copyAndInsertRow,
            insertInterpolation;

      public ButtonPanel()
      {
         JPanel buttonPanel = new JPanel();

         selectNewPoseSequence = new JButton("Select new pose sequence");
         selectPoseSequence = new JButton("Select pose sequence to append");
         deleteRow = new JButton("Delete");
         updateSCS = new JButton("Update SCS");
         setRowWithSlider = new JButton("Set row with slider");
         save = new JButton("Save");
         copyAndInsertRow = new JButton("Copy/insert row");
         switchSideDependentValues = new JButton("Switch side dependent values");
         insertInterpolation = new JButton("Insert interpolated row");

         buttonPanel.add(selectNewPoseSequence);
         buttonPanel.add(selectPoseSequence);
         buttonPanel.add(copyAndInsertRow);
         buttonPanel.add(insertInterpolation);
         buttonPanel.add(deleteRow);
         buttonPanel.add(updateSCS);
         buttonPanel.add(setRowWithSlider);
         buttonPanel.add(switchSideDependentValues);
         buttonPanel.add(save);

         selectNewPoseSequence.addActionListener(this);
         copyAndInsertRow.addActionListener(this);
         insertInterpolation.addActionListener(this);
         selectPoseSequence.addActionListener(this);
         deleteRow.addActionListener(this);
         updateSCS.addActionListener(this);
         setRowWithSlider.addActionListener(this);
         save.addActionListener(this);
         switchSideDependentValues.addActionListener(this);

         setLayout(new BorderLayout());
         add(buttonPanel, BorderLayout.SOUTH);

      }

      public void actionPerformed(ActionEvent e)
      {
         if (e.getSource().equals(selectPoseSequence))
            poseSequenceSelectorPanel.addSequenceFromFile();

         else if (e.getSource().equals(deleteRow))
            poseSequenceSelectorPanel.deleteSelectedRows();

         else if (e.getSource().equals(updateSCS))
            poseSequenceSelectorPanel.updateSCS();

         else if (e.getSource().equals(setRowWithSlider))
            poseSequenceSelectorPanel.setRowWithSlider();

         else if (e.getSource().equals(save))
            poseSequenceSelectorPanel.save();

         else if (e.getSource().equals(selectNewPoseSequence))
            poseSequenceSelectorPanel.newSequenceFromFile();

         else if (e.getSource().equals(copyAndInsertRow))
            poseSequenceSelectorPanel.copyAndInsertRow();

         else if (e.getSource().equals(insertInterpolation))
            poseSequenceSelectorPanel.insertInterpolation();

         else if (e.getSource().equals(switchSideDependentValues))
            poseSequenceSelectorPanel.switchSideDependentValues();
      }
   }

   public void setSequence(PlaybackPoseSequence seq)
   {
      poseSequenceSelectorPanel.setSequence(seq);
   }

   public void addSequence(PlaybackPoseSequence seq)
   {
      poseSequenceSelectorPanel.addSequence(seq);
   }

   private class JointNameKey extends JPanel
   {
      private static final long serialVersionUID = -986320074350226999L;

      public JointNameKey()
      {
         String key = "Joint name    =   (left, right)   +   (ankle, elbow, hip, knee, neck, shoulder, spine, wrist)   +   (yaw, pitch, roll)";
         JLabel jointNameKey = new JLabel(key);
         add("key", jointNameKey);
      }
   }

}
