package us.ihmc.darpaRoboticsChallenge.controllers;

import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepStatus;

public interface PathQueueManager
{

   public abstract void updateFootstepStatus(FootstepStatus footstepStatus);

   public abstract void executeFootstepQueue();

   public abstract void setPathReplacer(AbstractPathGenerator pathReplacer);

}
