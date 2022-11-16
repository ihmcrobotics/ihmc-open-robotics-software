package us.ihmc.avatar.sharedControl;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.ui.tools.TrajectoryRecordReplay;
import us.ihmc.tools.io.WorkspaceDirectory;

public class ProMPAssistantTest
{
   public static void main(String[] args)
   {
      // learn ProMPs
      // Check ProMPAssistant.json if you want to change parameters (e.g, task to learn, body parts to consider in the motion)
      ProMPAssistant proMPAssistant = new ProMPAssistant();
      // use a csv file with the trajectories of the hands of the robot for testing
      WorkspaceDirectory directory = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/etc");
      String directoryAbsolutePath = directory.getDirectoryPath().toAbsolutePath().toString();
      String demoDirectory = directoryAbsolutePath + "/test/PushDoorTest";
      String testFilePath = demoDirectory + "/1.csv";

      // replay that file
      TrajectoryRecordReplay<Double> trajectoryPlayer = new TrajectoryRecordReplay<>(Double.class, testFilePath, 2); //2 body parts: the hands
      FramePose3D leftHandFramePose = new FramePose3D();
      leftHandFramePose.setFromReferenceFrame(ReferenceFrame.getWorldFrame());
      FramePose3D rightHandFramePose = new FramePose3D();
      rightHandFramePose.setFromReferenceFrame(ReferenceFrame.getWorldFrame());
      trajectoryPlayer.setDoneReplay(false);

      TrajectoryRecordReplay<Double> trajectoryRecorder = new TrajectoryRecordReplay<>(Double.class, directoryAbsolutePath, 2); //2 body parts: the hands
      trajectoryRecorder.setRecordFileName("generatedMotion.csv");
      while (!trajectoryPlayer.hasDoneReplay())
      {
         // Read file with stored trajectories: read set point per timestep until file is over
         Double[] dataPoint = trajectoryPlayer.play();

         // [0,1,2,3] quaternion of body segment; [4,5,6] position of body segment
         // right hand +7. This is the way files are generated from recordings in VR. Check KinematicsRecordReplay
         leftHandFramePose.getOrientation().set(dataPoint[0], dataPoint[1], dataPoint[2], dataPoint[3]);
         leftHandFramePose.getPosition().set(dataPoint[4], dataPoint[5], dataPoint[6]);
         rightHandFramePose.getOrientation().set(dataPoint[7], dataPoint[8], dataPoint[9], dataPoint[10]);
         rightHandFramePose.getPosition().set(dataPoint[11], dataPoint[12], dataPoint[13]);

         if (proMPAssistant.readyToPack())
         {
            //change frame according to generated ProMP
            proMPAssistant.framePoseToPack(leftHandFramePose, "leftHand");
         }
         else
         {
            //do not change the frame, just observe it in order to generate a prediction later
            proMPAssistant.processFrameInformation(leftHandFramePose, "leftHand");
         }
         //record frame and store it in csv file
         Double[] leftHandTrajectories = new Double[] {leftHandFramePose.getOrientation().getX(),
                                                       leftHandFramePose.getOrientation().getY(),
                                                       leftHandFramePose.getOrientation().getZ(),
                                                       leftHandFramePose.getOrientation().getS(),
                                                       leftHandFramePose.getPosition().getX(),
                                                       leftHandFramePose.getPosition().getY(),
                                                       leftHandFramePose.getPosition().getZ()};
         trajectoryRecorder.record(leftHandTrajectories);
         // keep in two separated repeated 'if' clauses, if you do not want to update twice the same promp
         // ideally put this 'if' clause once in a 'for' loop that iterates over all body parts (see in RDXVRKinematicsStreamingMode)
         // Note. processFrameInformation() updates all proMPs right before being ready to pack
         if (proMPAssistant.readyToPack())
         {
            //change frame according to generated ProMP
            proMPAssistant.framePoseToPack(rightHandFramePose, "rightHand");
         }
         else
         {
            //do not change the frame, just observe it in order to generate a prediction later
            proMPAssistant.processFrameInformation(rightHandFramePose, "rightHand");
         }
         //record frame and store it in csv file
         Double[] rightHandTrajectories = new Double[] {rightHandFramePose.getOrientation().getX(),
                                                        rightHandFramePose.getOrientation().getY(),
                                                        rightHandFramePose.getOrientation().getZ(),
                                                        rightHandFramePose.getOrientation().getS(),
                                                        rightHandFramePose.getPosition().getX(),
                                                        rightHandFramePose.getPosition().getY(),
                                                        rightHandFramePose.getPosition().getZ()};
         trajectoryRecorder.record(rightHandTrajectories);
      }
      //concatenate each set point of hands in single row
      trajectoryRecorder.concatenateData();
      //save recording in csv file
      trajectoryRecorder.saveRecording();

      LogTools.info("Test completed successfully!");
      LogTools.info("You can visualize the ProMPs plots by running the file {}/1Dplots_ProMPAssistantTest.py", directoryAbsolutePath);
      LogTools.info("You can use file {}/{} as a replay file in Kinematics Streaming Mode",
                    directoryAbsolutePath,
                    trajectoryRecorder.getRecordFileName());
   }
}
