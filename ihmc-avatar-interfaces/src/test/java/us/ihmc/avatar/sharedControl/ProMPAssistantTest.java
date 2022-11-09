package us.ihmc.avatar.sharedControl;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.ui.tools.TrajectoryRecordReplay;
import us.ihmc.tools.io.WorkspaceDirectory;

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
      trajectoryPlayer.setDoneReplay(false);
      while (!trajectoryPlayer.hasDoneReplay())
      {
         // Read file with stored trajectories: read setpoint per timestep until file is over
         Double[] dataPoint = trajectoryPlayer.play();

         // [0,1,2,3] quaternion of body segment; [4,5,6] position of body segment
         framePose.getOrientation().set(dataPoint[7], dataPoint[8], dataPoint[9], dataPoint[10]);
         framePose.getPosition().set(dataPoint[11], dataPoint[12], dataPoint[13]);

         if (proMPAssistant.readyToPack())
            proMPAssistant.framePoseToPack(framePose, "rightHand");
         else
            proMPAssistant.processFrameInformation(framePose, "rightHand");
      }
   }
}
