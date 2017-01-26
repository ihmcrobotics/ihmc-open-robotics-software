package us.ihmc.avatar.posePlayback;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.util.ArrayList;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class PlaybackPoseSequenceReader
{
   public static void appendFromFile(PlaybackPoseSequence posePlaybackRobotPoseSequence, String fileName)
   {
      String fullFileName = PlaybackPoseSequenceWriter.directory + fileName;
      File file = new File(fullFileName);
      appendFromFile(posePlaybackRobotPoseSequence, file);
   }

   
   
   public static void appendFromFile(PlaybackPoseSequence posePlaybackRobotPoseSequence, InputStream selectedFile)
   {
      appendFromFile(posePlaybackRobotPoseSequence, new InputStreamReader(selectedFile));
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

         for (int i = 0; i < jointNamesArray.length; i++)
         {
            jointNames.add(jointNamesArray[i]);
         }

         if (!jointNames.get(0).equals("delayBeforePose"))
            throw new RuntimeException("Expecting delayBeforePose on first line. Got " + jointNames.get(0));
         jointNames.remove(0);

         if (!jointNames.get(0).equals("poseDuration"))
            throw new RuntimeException("Expecting poseDuration on first line. Got " + jointNames.get(0));
         jointNames.remove(0);

         OneDoFJoint[] allJoints = fullRobotModel.getOneDoFJoints();

         jointNamesArray = new String[jointNames.size()];
         jointNames.toArray(jointNamesArray);

         InverseDynamicsJoint[] inverseDynamicsJoints = ScrewTools.findJointsWithNames(allJoints, jointNamesArray);
         OneDoFJoint[] oneDoFJoints = ScrewTools.filterJoints(inverseDynamicsJoints, OneDoFJoint.class);
         double[] jointAngles = new double[oneDoFJoints.length];

         String textPose = br.readLine();    // read one line of text (one pose)

         while (textPose != null)    // check for end of file
         {
            String[] poseData = textPose.split("\\s+");    // separate each piece of data in one line
            PlaybackPose robotPose;

            if (oneDoFJoints.length + 2 != poseData.length)
               throw new RuntimeException("oneDoFJoints.length + 2 != poseData.length");

            double delay = Double.parseDouble(poseData[0]);
            double duration = Double.parseDouble(poseData[1]);

            for (int i = 0; i < oneDoFJoints.length; i++)
            {
               jointAngles[i] = Double.parseDouble(poseData[i + 2]);
            }

            robotPose = new PlaybackPose(oneDoFJoints, jointAngles, delay, duration);
            posePlaybackRobotPoseSequence.addPose(robotPose);

            textPose = br.readLine();    // read next pose
         }

      }
      catch (Exception ex)
      {
         ex.printStackTrace();
      }

   }

   public static PlaybackPoseSequence readFromFile(FullRobotModel fullRobotModel, String fileName)
   {
      PlaybackPoseSequence ret = new PlaybackPoseSequence(fullRobotModel);
      appendFromFile(ret, fileName);

      return ret;
   }

   public static PlaybackPoseSequence readFromInputStream(FullRobotModel fullRobotModel, InputStream inputStream)
   {
      PlaybackPoseSequence ret = new PlaybackPoseSequence(fullRobotModel);

      InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
      appendFromFile(ret, inputStreamReader);
  
      return ret;
   }

}
