package us.ihmc.quadrupedRobotics.input.mode;

import java.util.Map;

import net.java.games.input.Event;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;

public interface QuadrupedTeleopMode
{
   void onEntry();

   void update(Map<XBoxOneMapping, Double> channels, QuadrupedTaskSpaceEstimator.Estimates estimates);
   void onInputEvent(Map<XBoxOneMapping, Double> channels, QuadrupedTaskSpaceEstimator.Estimates estimates, Event event);

   void onExit();
}
