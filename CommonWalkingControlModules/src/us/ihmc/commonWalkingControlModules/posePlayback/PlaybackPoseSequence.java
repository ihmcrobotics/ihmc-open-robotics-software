package us.ihmc.commonWalkingControlModules.posePlayback;

import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.io.Reader;
import java.util.ArrayList;
import java.util.List;

import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JRadioButton;
import javax.swing.JTextField;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.posePlayback.PlaybackPose;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.ScrewTools;

public class PlaybackPoseSequence
{
   private static final String directory = "PoseSequences/";
   private final ArrayList<PlaybackPose> poseSequence = new ArrayList<PlaybackPose>();

   private final FullRobotModel fullRobotModel;
   
   public PlaybackPoseSequence(FullRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
   }

   public PlaybackPoseSequence(FullRobotModel fullRobotModel, String fileName)
   {
      this.fullRobotModel = fullRobotModel;
      appendFromFile(this, fileName);
   }
   
   public PlaybackPoseSequence(FullRobotModel fullRobotModel, InputStream inputStream)
   {
      this.fullRobotModel = fullRobotModel;

      InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
      appendFromFile(this, inputStreamReader);
      try
      {
         inputStreamReader.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void addPose(PlaybackPose posePlaybackRobotPose)
   {
      poseSequence.add(posePlaybackRobotPose);
   }

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
      //TODO make this work with saving/reading finger poses as well

      if ((fileName != null) && !fileName.endsWith(".poseSequence"))
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
         //save delay parameter
         printWriter.print(String.format("%.3f \t", posePlaybackRobotPose.getPlayBackDelayBeforePose()));
         printWriter.print(String.format("%.3f \t", posePlaybackRobotPose.getPlayBackDuration()));

         //save body pose
         posePlaybackRobotPose.getJointAngles(oneDoFJoints, jointAngles);
         
         for (Double jointAngle : jointAngles)
         {
            printWriter.print(String.format("%.3f \t", jointAngle));
         }

         
         printWriter.println();
      }

      printWriter.close();
   }

   public static void appendFromFile(PlaybackPoseSequence posePlaybackRobotPoseSequence, String fileName)
   {
      String fullFileName = directory + fileName;
      File file = new File(fullFileName);
      appendFromFile(posePlaybackRobotPoseSequence, file);
   }

   public static void appendFromFile(PlaybackPoseSequence posePlaybackRobotPoseSequence, File selectedFile)
   {
      try
      {
         FileReader fr = new FileReader(selectedFile);
         appendFromFile(posePlaybackRobotPoseSequence, fr);
         fr.close();
         
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
   
   public static void appendFromFile(PlaybackPoseSequence posePlaybackRobotPoseSequence, Reader reader)
   {
      FullRobotModel fullRobotModel = posePlaybackRobotPoseSequence.getFullRobotModel();
      
      try
      {
         BufferedReader br = new BufferedReader(reader);
         
         String jointNamesOnOneLine = br.readLine();
         String[] jointNamesArray = jointNamesOnOneLine.split("\\s+");
         ArrayList<String> jointNames = new ArrayList<String>();
         
         for (int i=0; i<jointNamesArray.length; i++)
         {
            jointNames.add(jointNamesArray[i]);
         }
         
         if (!jointNames.get(0).equals("delayBeforePose")) throw new RuntimeException("Expecting delayBeforePose on first line. Got " + jointNames.get(0));
         jointNames.remove(0);
         
         if (!jointNames.get(0).equals("poseDuration")) throw new RuntimeException("Expecting poseDuration on first line. Got " + jointNames.get(0));
         jointNames.remove(0);
         
         OneDoFJoint[] allJoints = fullRobotModel.getOneDoFJoints();
         
         jointNamesArray = new String[jointNames.size()];
         jointNames.toArray(jointNamesArray);
         
         InverseDynamicsJoint[] inverseDynamicsJoints = ScrewTools.findJointsWithNames(allJoints, jointNamesArray);
         OneDoFJoint[] oneDoFJoints = ScrewTools.filterJoints(inverseDynamicsJoints, OneDoFJoint.class);
         double[] jointAngles = new double[oneDoFJoints.length];
         
         String textPose = br.readLine(); // read one line of text (one pose)

         while (textPose != null) // check for end of file
         {
            String[] poseData = textPose.split("\\s+"); // separate each piece of data in one line
            PlaybackPose robotPose;

            if (oneDoFJoints.length + 2 != poseData.length) throw new RuntimeException("oneDoFJoints.length + 2 != poseData.length");

            double delay = Double.parseDouble(poseData[0]);
            double duration = Double.parseDouble(poseData[1]);
            
           for (int i=0; i<oneDoFJoints.length; i++)
           {
              jointAngles[i] = Double.parseDouble(poseData[i+2]);
           }

            robotPose = new PlaybackPose(oneDoFJoints, jointAngles, delay, duration);
            posePlaybackRobotPoseSequence.addPose(robotPose);

            textPose = br.readLine();//read next pose
         }

      }
      catch (Exception ex)
      {
         ex.printStackTrace();
      }

   }

   public FullRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public ArrayList<PlaybackPose> getPoseSequence()
   {
      return poseSequence;
   }

   public PlaybackPose getPose(int index)
   {
      if(index<poseSequence.size())
         return poseSequence.get(index);
      else
         return poseSequence.get(poseSequence.size()-1);
   }

   public PlaybackPose getFinalPose()
   {
      return getPose(getNumberOfPoses() - 1);
   }

   public int getNumberOfPoses()
   {
      return poseSequence.size();
   }

   public void clear()
   {
      poseSequence.clear();
   }

   public boolean epsilonEquals(PlaybackPoseSequence otherPoseSequence, double jointEpsilon, double timeEpsilon)
   {
      ArrayList<PlaybackPose> otherPoseArray = otherPoseSequence.getPoseSequence();

      if (otherPoseArray.size() != this.poseSequence.size())
         return false;

      for (int i = 0; i < poseSequence.size(); i++)
      {
         PlaybackPose thisPose = poseSequence.get(i);
         PlaybackPose otherPose = otherPoseArray.get(i);

         if (thisPose.epsilonEquals(otherPose, jointEpsilon, timeEpsilon))
            return false;
      }

      return true;
   }
   
   public PlaybackPoseSequence copy()
   {
      PlaybackPoseSequence posePlaybackRobotPoseSequence = new PlaybackPoseSequence(fullRobotModel);
      for(PlaybackPose pose : poseSequence)
      {
         posePlaybackRobotPoseSequence.addPose(pose.copy());
      }

      return posePlaybackRobotPoseSequence;
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
               
            case STAND_FROM_BACK:
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
