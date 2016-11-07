package us.ihmc.avatar.posePlayback;

import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedOutputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JRadioButton;
import javax.swing.JTextField;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class PlaybackPoseSequenceWriter
{
   protected static final String directory = "PoseSequences/";

   public static void promptWriteToFile(final PlaybackPoseSequence posePlaybackRobotPoseSequence)
   {
      final JFrame saveFrame = new JFrame("Save sequence");
      saveFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
      saveFrame.setLayout(new FlowLayout());
      saveFrame.setAlwaysOnTop(true);

      final ButtonGroup buttonGroup = new ButtonGroup();
      final List<JRadioButton> buttonList = new ArrayList<JRadioButton>();

      for (PosePlaybackSequenceType sequenceType : PosePlaybackSequenceType.values())
      {
         final JRadioButton typeButton = new JRadioButton(sequenceType.toString());
         saveFrame.add(typeButton);
         buttonGroup.add(typeButton);
         buttonList.add(typeButton);
      }

      final JTextField fileNameInput = new JTextField("Type file name");
      saveFrame.add(fileNameInput);

      JButton saveButton = new JButton("Save");
      saveFrame.add(saveButton);
      saveFrame.pack();
      saveFrame.setVisible(true);
      saveButton.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent arg0)
         {
            String sequenceDirectory = "";
            for (JRadioButton button : buttonList)
            {
               if (button.isSelected())
                  sequenceDirectory = button.getText() + "/";
            }

            writeToFile(posePlaybackRobotPoseSequence, sequenceDirectory + fileNameInput.getText());
            saveFrame.dispose();
         }
      });
   }

   public static void writeToFile(PlaybackPoseSequence posePlaybackRobotPoseSequence, String fileName)
   {
      // TODO make this work with saving/reading finger poses as well

      if ((fileName != null) &&!fileName.endsWith(".poseSequence"))
      {
         fileName = fileName + ".poseSequence";
      }

      String fullFileName = directory + fileName;

      try
      {
         FileOutputStream fileOutputStream = new FileOutputStream(fullFileName);
         writeToOutputStream(posePlaybackRobotPoseSequence, fileOutputStream);
      }
      catch (FileNotFoundException ex)
      {
         ex.printStackTrace();
      }

   }

   public static void writeToOutputStream(PlaybackPoseSequence posePlaybackRobotPoseSequence, OutputStream outputStream)
   {
      PrintWriter printWriter = new PrintWriter(new BufferedOutputStream(outputStream));

      ArrayList<PlaybackPose> poseSequence = posePlaybackRobotPoseSequence.getPoseSequence();

      PlaybackPose firstPose = poseSequence.get(0);
      ArrayList<OneDoFJoint> oneDoFJoints = new ArrayList<>();
      firstPose.getOneDoFJoints(oneDoFJoints);

      printWriter.print("delayBeforePose poseDuration");

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         printWriter.print(" " + oneDoFJoint.getName());
      }

      printWriter.println();

      double[] jointAngles = new double[oneDoFJoints.size()];

      for (PlaybackPose posePlaybackRobotPose : poseSequence)
      {
         // save delay parameter
         printWriter.print(String.format("%.3f \t", posePlaybackRobotPose.getPlayBackDelayBeforePose()));
         printWriter.print(String.format("%.3f \t", posePlaybackRobotPose.getPlayBackDuration()));

         // save body pose
         posePlaybackRobotPose.getJointAngles(oneDoFJoints, jointAngles);

         for (Double jointAngle : jointAngles)
         {
            printWriter.print(String.format("%.3f \t", jointAngle));
         }


         printWriter.println();
      }

      printWriter.close();
   }
   
   
   public enum PosePlaybackSequenceType
   {
      // NOTE: make a directory in /DarpaRoboticsChallenge/PoseSequences whose name is the toString when adding a type

      FRONT_TO_BACK, BACK_TO_FRONT, STAND_FROM_BACK, STAND_FROM_FRONT, OTHER, CAR_INGRESS, CAR_EGRESS;

      @Override
      public String toString()
      {
         String out = "";
         switch (this)
         {
            case FRONT_TO_BACK :
               out = "FrontToBack";

               break;

            case BACK_TO_FRONT :
               out = "BackToFront";

               break;

            case STAND_FROM_BACK :
               out = "StandFromBack";

               break;

            case STAND_FROM_FRONT :
               out = "StandFromFront";

               break;

            case OTHER :
               out = "Other";

               break;

            case CAR_INGRESS :
               out = "CarIngress";

               break;

            case CAR_EGRESS :
               out = "CarEgress";

               break;
         }

         return out;
      }
   }
}
