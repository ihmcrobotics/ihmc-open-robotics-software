package us.ihmc.quadrupedRobotics.input.mode;

import java.util.Map;

import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.input.InputChannel;
import us.ihmc.quadrupedRobotics.input.InputEvent;

public interface QuadrupedTeleopMode
{
   void onEntry();

   void update(Map<InputChannel, Double> channels, QuadrupedTaskSpaceEstimator.Estimates estimates);
   void onInputEvent(Map<InputChannel, Double> channels, QuadrupedTaskSpaceEstimator.Estimates estimates, InputEvent e);

   void onExit();

}
