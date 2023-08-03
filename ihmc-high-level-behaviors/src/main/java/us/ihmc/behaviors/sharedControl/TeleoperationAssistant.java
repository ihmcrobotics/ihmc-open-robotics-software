package us.ihmc.behaviors.sharedControl;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;

public interface TeleoperationAssistant
{
   void processFrameInformation(Pose3DReadOnly bodyPartObservedPose, String bodyPart);

   boolean readyToPack();

   void framePoseToPack(FramePose3D framePose, String bodyPart);
}
