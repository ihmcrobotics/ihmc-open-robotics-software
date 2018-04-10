package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import controller_msgs.msg.dds.FootstepDataListMessage;

public interface FootstepMessenger
{
   void submitFootsteps(FootstepDataListMessage footsteps);
}
