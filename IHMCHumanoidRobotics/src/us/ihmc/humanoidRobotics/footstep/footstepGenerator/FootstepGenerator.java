package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import java.util.Collection;

import us.ihmc.humanoidRobotics.footstep.Footstep;

public interface FootstepGenerator
{
   Collection<? extends Footstep> generateDesiredFootstepList();
}
