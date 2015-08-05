package us.ihmc.darpaRoboticsChallenge.controllers;

import java.util.ArrayList;

import us.ihmc.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.footstep.Footstep;

public interface PathQueueManager
{
   public abstract void updateFootstepStatus(FootstepStatus footstepStatus);

   public abstract void setPathReplacer(Path pathReplacer);

   public abstract ArrayList<Footstep> getFootStepQueue();

}
