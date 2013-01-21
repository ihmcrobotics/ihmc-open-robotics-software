package us.ihmc.darpaRoboticsChallenge.controllers;

import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepStatus;

public interface PathManager
{

   public abstract void updateFootstepStatus(FootstepStatus footstepStatus);

   public abstract void executeFootstepQueue();

   public abstract void setPathReplacer(AbstractFootstepGenerator pathReplacer);

   public abstract void generatePath();

   public abstract void unPause();

   public abstract void pause();

}
