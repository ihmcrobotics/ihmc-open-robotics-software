package us.ihmc.aware.controller.force.taskSpaceController;

public interface QuadrupedTaskSpaceFeedbackBlock
{
   void reset();
   void compute(QuadrupedTaskSpaceEstimates estimates, QuadrupedTaskSpaceCommands commands);
}
