package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;

public interface FootstepAdjustment
{
   FramePose3DReadOnly adjustFootstep(FramePose2DReadOnly footstepPose);
}
