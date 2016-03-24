package us.ihmc.aware.controller.force.taskSpaceController.feedbackController;

import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceSetpoints;

public interface QuadrupedTaskSpaceFeedbackController
{
   void reset();
   void computeFeedback(QuadrupedTaskSpaceEstimates estimates, QuadrupedTaskSpaceSetpoints setpoints, QuadrupedTaskSpaceCommands feedbackCommands);
   void enableVisualization(boolean enable);
}
