package us.ihmc.darpaRoboticsChallenge.controllers;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.communication.packets.walking.FootstepStatus;

public interface PathQueueManager
{
   public abstract void updateFootstepStatus(FootstepStatus footstepStatus);

   public abstract void setPathReplacer(Path pathReplacer);

   public abstract ArrayList<Footstep> getFootStepQueue();

}
