package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import java.util.ArrayList;

import us.ihmc.humanoidRobotics.footstep.Footstep;

public interface FootstepGenerator
{
   ArrayList<Footstep> generateDesiredFootstepList();
}
