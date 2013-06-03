package us.ihmc.darpaRoboticsChallenge.posePlayback;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

public class PoseSequenceEditorGUI extends JFrame
{
   private final PoseSequenceSelectorPanel poseSequenceSelectorPanel;
   private final ButtonPanel buttonPanel;

   public PoseSequenceEditorGUI()
   {
      super("Pose sequence editor");
      setSize(1400, 600);
      poseSequenceSelectorPanel = new PoseSequenceSelectorPanel();
      buttonPanel = new ButtonPanel();
      
      getContentPane().add(new JointNameKey(), BorderLayout.NORTH);
      getContentPane().add(poseSequenceSelectorPanel, BorderLayout.CENTER);
      getContentPane().add(buttonPanel, BorderLayout.SOUTH);
   }
   
   private class ButtonPanel extends JPanel implements ActionListener 
   { 
      private JButton selectPoseSequence, deleteRow, updateSCS, save; 
      
      public ButtonPanel()
      {
         JPanel buttonPanel = new JPanel();
         
         selectPoseSequence = new JButton("Select pose sequence");
         deleteRow = new JButton("Delete"); 
         updateSCS = new JButton("Update SCS");
         save = new JButton("Save");
         
         buttonPanel.add(selectPoseSequence);
         buttonPanel.add(deleteRow);
         buttonPanel.add(updateSCS);
         buttonPanel.add(save);
         
         selectPoseSequence.addActionListener(this);
         deleteRow.addActionListener(this);
         updateSCS.addActionListener(this);
         save.addActionListener(this);
                  
         setLayout(new BorderLayout());
         add(buttonPanel, BorderLayout.SOUTH);

      }

      public void actionPerformed(ActionEvent e)
      {
         if(e.getSource().equals(selectPoseSequence))
            poseSequenceSelectorPanel.addSequence();
         else if(e.getSource().equals(deleteRow))
            poseSequenceSelectorPanel.deleteSelectedRows();
         else if(e.getSource().equals(updateSCS))
            poseSequenceSelectorPanel.updateSCS();
         else if(e.getSource().equals(save))
            save();
      }
      
      private void save()
      {
      }
   }
   
   private class JointNameKey extends JPanel
   {
      public JointNameKey()
      {
         String key = "Joint name    =   (left, right)   +   (ankle, elbow, hip, knee, neck, shoulder, spine, wrist)   +   (yaw, pitch, roll)";
         JLabel jointNameKey = new JLabel(key);
         add("key", jointNameKey);
      }
   }

   
   public static void main(String[] args)
   {
      PoseSequenceEditorGUI scriptedEditorGUI = new PoseSequenceEditorGUI();
      scriptedEditorGUI.setDefaultCloseOperation(EXIT_ON_CLOSE);
      scriptedEditorGUI.setVisible(true);
   }
}

