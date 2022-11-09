package us.ihmc.avatar.sharedControl;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.ui.tools.TrajectoryRecordReplay;
import us.ihmc.tools.io.WorkspaceDirectory;

import java.io.File;

public class ProMPAssistantTest
{
   public static void main(String[] args)
   {
      ProMPAssistant proMPAssistant = new ProMPAssistant();

      WorkspaceDirectory directory = new WorkspaceDirectory("ihmc-open-robotics-software", "promp/etc");
      String directoryAbsolutePath = directory.getDirectoryPath().toAbsolutePath().toString();
      String demoDirectory = directoryAbsolutePath + "/test/PushDoorTest";
      String testFilePath = demoDirectory + "/1.csv";

      TrajectoryRecordReplay<Double> trajectoryPlayer = new TrajectoryRecordReplay<>(Double.class, testFilePath, 2);
      FramePose3D framePose = new FramePose3D();
      framePose.setFromReferenceFrame(ReferenceFrame.getWorldFrame());
      while(!trajectoryPlayer.hasDoneReplay())
      {
         // Read file with stored trajectories: read setpoint per timestep until file is over
         Double[] dataPoint = trajectoryPlayer.play();
         // [0,1,2,3] quaternion of body segment; [4,5,6] position of body segment
         framePose.getOrientation().set(dataPoint[0], dataPoint[1], dataPoint[2], dataPoint[3]);
         framePose.getPosition().set(dataPoint[4], dataPoint[5], dataPoint[6]);

         if(proMPAssistant.readyToPack())
            proMPAssistant.framePoseToPack(framePose, "rightHand");
         else
            proMPAssistant.processFrameInformation(framePose, "rightHand");
      }
   }
}
