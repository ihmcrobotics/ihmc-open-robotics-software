package us.ihmc.avatar.sharedControl;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
      TrajectoryRecordReplay<Double> trajectoryPlayer = new TrajectoryRecordReplay<>(Double.class, testFilePath, 2);
      FramePose3D framePose = new FramePose3D();
      framePose.setFromReferenceFrame(ReferenceFrame.getWorldFrame());
      trajectoryPlayer.setDoneReplay(false);
      while (!trajectoryPlayer.hasDoneReplay())
      {
         // Read file with stored trajectories: read setpoint per timestep until file is over
         Double[] dataPoint = trajectoryPlayer.play();

         // [0,1,2,3] quaternion of body segment; [4,5,6] position of body segment
         // right hand +7. This is the way files are generated from recordings in VR. Check KinematicsRecordReplay
         framePose.getOrientation().set(dataPoint[7], dataPoint[8], dataPoint[9], dataPoint[10]);
         framePose.getPosition().set(dataPoint[11], dataPoint[12], dataPoint[13]);

         if (proMPAssistant.readyToPack()){
            //change frame according to generated ProMP
            proMPAssistant.framePoseToPack(framePose, "rightHand");
         }
         else
         {
            //do not change the frame, just observe it in order to later generate a prediction
            proMPAssistant.processFrameInformation(framePose, "rightHand");
         }
      }
   }
}
